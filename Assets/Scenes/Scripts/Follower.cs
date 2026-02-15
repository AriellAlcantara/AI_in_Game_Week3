using System.Collections;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public class Follower : MonoBehaviour
{
    [Header("Core")]
    public string playerTag = "Player";
    public Transform player;

    [Header("Levels (low -> high)")]
    // Provide one anchor per logical level (low, mid, high...). Must be ordered or will be sorted by Y.
    public Transform[] levelAnchors;

    [Header("Movement")]
    public float repathInterval = 0.15f; // seconds between SetDestination calls
    public float linkTraverseSpeed = 3f;
    public float linkTraverseHeight = 1.0f;

    [Header("Stopping")]
    // Distance in meters to stop from the player to avoid pushing them
    public float stopDistance = 2f;

    [Header("Animation (optional)")]
    public Animator animator;
    public string runningBool = "Running";

    private NavMeshAgent agent;
    private float lastRepathTime = -999f;

    // cached OffMeshLink components in scene
    private OffMeshLink[] offMeshLinks;

    void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        agent.autoTraverseOffMeshLink = false; // we will handle traversal
        if (player == null)
        {
            try
            {
                var go = GameObject.FindGameObjectWithTag(playerTag);
                if (go) player = go.transform;
            }
            catch (UnityException)
            {
                // ignore
            }
        }

        // sort anchors by Y so level order is low->high
        if (levelAnchors != null && levelAnchors.Length > 1)
        {
            System.Array.Sort(levelAnchors, (a, b) => a.position.y.CompareTo(b.position.y));
        }

        // ensure agent stopping distance aligns with desired stopDistance
        agent.stoppingDistance = stopDistance;

        // cache OffMeshLinks
        offMeshLinks = FindObjectsOfType<OffMeshLink>();
    }

    void Update()
    {
        if (player == null) return;

        // Off-mesh link traversal handling
        if (agent.isOnOffMeshLink)
        {
            if (!isTraversing)
                StartCoroutine(TraverseOffMeshLink());
            return; // wait while traversing
        }

        // Periodically update destination
        if (Time.time - lastRepathTime > repathInterval)
        {
            lastRepathTime = Time.time;
            Vector3 target = GetNextTargetPosition();

            // Compute horizontal distance to target (ignore Y) to decide stopping
            Vector3 toTarget = target - transform.position;
            toTarget.y = 0f;
            float sqrDist = toTarget.sqrMagnitude;
            float sqrStop = stopDistance * stopDistance;

            if (sqrDist <= sqrStop)
            {
                // close enough -> stop to avoid pushing the player
                if (!agent.isStopped)
                    agent.isStopped = true;
                agent.ResetPath();
            }
            else
            {
                if (agent.isStopped)
                    agent.isStopped = false;
                SetDestinationSafe(target);
            }
        }

        // animation
        if (animator)
        {
            bool moving = agent.hasPath && agent.velocity.sqrMagnitude > 0.01f;
            animator.SetBool(runningBool, moving);
        }
    }

    private Vector3 GetNextTargetPosition()
    {
        // Determine level indices using nearest anchor Y
        int agentLevel = GetLevelIndex(transform.position);
        int playerLevel = GetLevelIndex(player.position);

        // If player is on a higher level than agent, move to the next intermediate level (agentLevel+1) via OffMeshLink if possible.
        if (playerLevel > agentLevel && levelAnchors != null && levelAnchors.Length > agentLevel + 0)
        {
            int nextLevel = Mathf.Clamp(agentLevel + 1, 0, levelAnchors.Length - 1);

            // try to find an OffMeshLink that connects agentLevel -> nextLevel
            var link = FindOffMeshLinkBetweenLevels(agentLevel, nextLevel);
            if (link != null)
            {
                // pick the endpoint on the agent's level as the approach target
                Vector3 endpoint = (GetLevelIndex(link.startTransform.position) == agentLevel) ? link.startTransform.position : link.endTransform.position;

                // Sample the NavMesh near the endpoint so the agent moves to a reachable point on the NavMesh
                if (SampleNavMesh(endpoint, out Vector3 sampled, 1.0f))
                    return sampled;

                // fallback to raw endpoint if sampling fails
                return endpoint;
            }

            // if no link found, fall back to moving to the next level anchor position (player XZ)
            if (levelAnchors != null && nextLevel < levelAnchors.Length)
            {
                Vector3 anchorPos = levelAnchors[nextLevel].position;
                Vector3 playerXZ = new Vector3(player.position.x, anchorPos.y, player.position.z);
                // sample that position too
                if (SampleNavMesh(playerXZ, out Vector3 sampledAnchor, 2f))
                    return sampledAnchor;
                return playerXZ;
            }
        }

        // If player is on a lower level than agent, do NOT enforce intermediate-step rule.
        // Allow direct chase to player's sampled NavMesh position so the agent can drop down freely.
        if (playerLevel < agentLevel)
        {
            if (SampleNavMesh(player.position, out Vector3 sampledPlayerDown, 2f))
                return sampledPlayerDown;
            return player.position;
        }

        // Otherwise (same level), directly chase the player (sampled)
        if (SampleNavMesh(player.position, out Vector3 sampledPlayer, 1.5f))
            return sampledPlayer;

        return player.position;
    }

    // Sample nearest NavMesh position within maxDistance
    private bool SampleNavMesh(Vector3 sourcePos, out Vector3 result, float maxDistance = 1f)
    {
        NavMeshHit hit;
        if (NavMesh.SamplePosition(sourcePos, out hit, maxDistance, NavMesh.AllAreas))
        {
            result = hit.position;
            return true;
        }
        result = sourcePos;
        return false;
    }

    private OffMeshLink FindOffMeshLinkBetweenLevels(int fromLevel, int toLevel)
    {
        if (offMeshLinks == null || offMeshLinks.Length == 0) return null;
        OffMeshLink best = null;
        float bestDist = float.MaxValue;

        for (int i = 0; i < offMeshLinks.Length; i++)
        {
            var link = offMeshLinks[i];
            if (link == null) continue;

            if (link.startTransform == null || link.endTransform == null) continue;

            int aLevel = GetLevelIndex(link.startTransform.position);
            int bLevel = GetLevelIndex(link.endTransform.position);

            // check if this link connects the requested levels in either direction
            bool matches = (aLevel == fromLevel && bLevel == toLevel) || (aLevel == toLevel && bLevel == fromLevel);
            if (!matches) continue;

            // prefer the link whose endpoint on fromLevel is closest to the agent
            Vector3 endpoint = (GetLevelIndex(link.startTransform.position) == fromLevel) ? link.startTransform.position : link.endTransform.position;
            float d = (new Vector3(endpoint.x, 0f, endpoint.z) - new Vector3(transform.position.x, 0f, transform.position.z)).sqrMagnitude;
            if (d < bestDist)
            {
                bestDist = d;
                best = link;
            }
        }

        return best;
    }

    private int GetLevelIndex(Vector3 pos)
    {
        if (levelAnchors == null || levelAnchors.Length == 0) return 0;
        float bestDist = float.MaxValue;
        int best = 0;
        for (int i = 0; i < levelAnchors.Length; i++)
        {
            float d = Mathf.Abs(pos.y - levelAnchors[i].position.y);
            if (d < bestDist)
            {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    private void SetDestinationSafe(Vector3 pos)
    {
        if (!agent.isOnNavMesh) return;
        agent.SetDestination(pos);
        // Debug when path becomes partial/invalid
        if (agent.pathStatus == NavMeshPathStatus.PathPartial || agent.pathStatus == NavMeshPathStatus.PathInvalid)
        {
            Debug.Log($"[Follower] PathStatus {agent.pathStatus} when setting destination to {pos}");
        }
    }

    private bool isTraversing = false;
    private IEnumerator TraverseOffMeshLink()
    {
        isTraversing = true;
        OffMeshLinkData data = agent.currentOffMeshLinkData;
        Vector3 startPos = agent.transform.position;
        Vector3 endPos = data.endPos + Vector3.up * agent.baseOffset;

        float duration = Vector3.Distance(startPos, endPos) / Mathf.Max(0.1f, linkTraverseSpeed);
        float t = 0f;

        agent.updatePosition = false;

        while (t < duration)
        {
            float frac = t / duration;
            Vector3 pos = Vector3.Lerp(startPos, endPos, frac);
            pos.y += linkTraverseHeight * 4f * (frac - frac * frac);
            agent.transform.position = pos;
            t += Time.deltaTime;
            yield return null;
        }

        agent.transform.position = endPos;

        agent.CompleteOffMeshLink();
        agent.updatePosition = true;
        isTraversing = false;

        Debug.Log("[Follower] Completed OffMeshLink traversal.");
    }
}

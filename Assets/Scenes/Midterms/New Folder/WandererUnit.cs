using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public class WandererUnit : MonoBehaviour
{
    private ILeader leader;
    private NavMeshAgent agent;

    [Header("Wandering")]
    public float roamRadius = 5f;
    public float roamInterval = 3f;
    public float sampleDistance = 2f;

    [Header("Movement")]
    public float moveSpeed = 3.5f; // editable speed for wanderers
    public float acceleration = 8f;
    public float angularSpeed = 120f;

    [Header("Follow")]
    public float followSpeedMultiplier = 1.0f; // multiplier when following a leader

    [Header("Collection")]
    public float collectRange = 1.2f; // local check radius to auto-become follower

    [Header("Separation")]
    public float separationRadius = 1.5f; // radius to detect nearby wanderers for separation
    public float separationStrength = 2.0f; // how strongly to push away when choosing roam targets
    public float separationApplyThreshold = 0.15f; // minimal separation vector magnitude to apply correction

    [Header("Visuals")]
    public Renderer targetRenderer; // optional renderer to color when collected
    private Material originalMaterial;

    private float lastRoamTime = -999f;
    private Vector3 roamCenter;

    // follow offset when attached to leader
    private Vector3 followOffset = Vector3.zero;

    void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        roamCenter = transform.position;
        if (agent != null)
        {
            agent.speed = moveSpeed;
            agent.acceleration = acceleration;
            agent.angularSpeed = angularSpeed;
            // ensure agent rotates to face movement
            agent.updateRotation = true;
            agent.updateUpAxis = true;
            // encourage local avoidance
            agent.avoidancePriority = Random.Range(10, 80);
        }

        if (targetRenderer == null)
            targetRenderer = GetComponentInChildren<Renderer>();
        if (targetRenderer != null)
            originalMaterial = targetRenderer.sharedMaterial;
    }

    void Update()
    {
        // If unclaimed, allow proximity-based pickup by leaders/player
        if (leader == null)
        {
            TryAutoClaim();
        }

        if (leader != null)
        {
            // follow leader
            var leaderMb = leader as MonoBehaviour;
            if (leaderMb != null && agent.isOnNavMesh)
            {
                // apply follow speed multiplier
                agent.speed = moveSpeed * followSpeedMultiplier;
                Vector3 target = leaderMb.transform.position + followOffset;
                agent.SetDestination(target);
            }
            return;
        }

        // wandering behavior when unclaimed
        if (!agent.isOnNavMesh) return;

        // ensure base movement settings while roaming
        agent.speed = moveSpeed;
        agent.acceleration = acceleration;
        agent.angularSpeed = angularSpeed;
        agent.updateRotation = true;

        // immediate separation if too close
        Vector3 immediateSeparation = ComputeSeparationVector();
        if (immediateSeparation.sqrMagnitude > 0.001f && agent.remainingDistance < 0.5f)
        {
            Vector3 nudgeTarget = transform.position + immediateSeparation.normalized * Mathf.Max(0.5f, separationRadius);
            if (NavMesh.SamplePosition(nudgeTarget, out NavMeshHit hitN, sampleDistance + 1f, NavMesh.AllAreas))
            {
                agent.SetDestination(hitN.position);
                return;
            }
        }

        // while moving, apply local separation correction so agents avoid heading into crowds
        if (agent.hasPath && agent.remainingDistance > 0.2f)
        {
            Vector3 sep = ComputeSeparationVector();
            if (sep.magnitude > separationApplyThreshold)
            {
                // compute an adjusted destination that nudges away from neighbors
                Vector3 adjusted = agent.destination + sep.normalized * separationStrength;
                if (NavMesh.SamplePosition(adjusted, out NavMeshHit adjHit, sampleDistance * 2f, NavMesh.AllAreas))
                {
                    agent.SetDestination(adjHit.position);
                }
            }
        }

        if (Time.time - lastRoamTime > roamInterval || !agent.hasPath || agent.remainingDistance < 0.5f)
        {
            lastRoamTime = Time.time;
            Vector2 r = Random.insideUnitCircle * roamRadius;
            Vector3 samplePos = roamCenter + new Vector3(r.x, 0f, r.y);

            // Add separation offset so chosen roam positions avoid nearby wanderers
            Vector3 sep = ComputeSeparationVector();
            samplePos += sep * separationStrength;

            if (NavMesh.SamplePosition(samplePos, out NavMeshHit hit, sampleDistance, NavMesh.AllAreas))
            {
                agent.SetDestination(hit.position);
            }
            else
            {
                // try fallback near current position
                if (NavMesh.SamplePosition(transform.position, out hit, roamRadius, NavMesh.AllAreas))
                    agent.SetDestination(hit.position);
            }
        }
    }

    // Compute a separation vector away from nearby wanderers (and other obstacles if desired)
    private Vector3 ComputeSeparationVector()
    {
        Vector3 sep = Vector3.zero;
        Collider[] hits = Physics.OverlapSphere(transform.position, separationRadius);
        foreach (var col in hits)
        {
            if (col == null) continue;
            if (col.gameObject == this.gameObject) continue;
            var other = col.GetComponentInParent<WandererUnit>();
            if (other == null) continue;
            if (other == this) continue;

            Vector3 toOther = transform.position - other.transform.position;
            float dist = toOther.magnitude;
            if (dist <= 0f) continue;
            // stronger repulsion when closer (inverse-square like)
            float weight = Mathf.Clamp01((separationRadius - dist) / separationRadius);
            sep += toOther.normalized * (weight / Mathf.Max(0.1f, dist));
        }
        return sep;
    }

    // Check nearby colliders for leaders/player and auto-claim
    private void TryAutoClaim()
    {
        Collider[] hits = Physics.OverlapSphere(transform.position, collectRange);
        foreach (var col in hits)
        {
            if (col == null) continue;
            // check for OpponentAI in parent chain
            var opp = col.GetComponentInParent<OpponentAI>();
            if (opp != null)
            {
                SetLeader(opp);
                return;
            }
            var pf = col.GetComponentInParent<PlayerFollowers>();
            if (pf != null)
            {
                SetLeader(pf);
                return;
            }
        }
    }

    // Also accept trigger-based pickup if colliders/rb set up
    private void OnTriggerEnter(Collider other)
    {
        if (leader != null) return;
        var opp = other.GetComponentInParent<OpponentAI>();
        if (opp != null)
        {
            SetLeader(opp);
            return;
        }
        var pf = other.GetComponentInParent<PlayerFollowers>();
        if (pf != null)
        {
            SetLeader(pf);
            return;
        }
    }

    public bool HasLeader() => leader != null;

    // Called by a leader to claim this unit
    public void SetLeader(ILeader l)
    {
        if (l == null) return;
        if (leader != null && leader == l) return; // already set to same leader

        // if already have a leader, release from it first
        if (leader != null && leader != l)
        {
            leader.RemoveFollower(this as MonoBehaviour);
        }

        leader = l;
        // notify leader to add (leader responsible for tracking followers)
        leader.AddFollower(this as MonoBehaviour);

        // choose an offset to avoid stacking
        Vector2 r = Random.insideUnitCircle * 1f;
        followOffset = new Vector3(r.x, 0f, r.y);

        // Do NOT parent NavMeshAgent under leader: agents should remain at root for NavMesh to work correctly.

        // ensure agent is active and adjust speed
        if (agent.isOnNavMesh)
        {
            agent.isStopped = false;
            agent.speed = moveSpeed * followSpeedMultiplier;
            agent.updateRotation = true;
        }

        // Apply leader material if leader provides one
        var mat = leader.GetLeaderMaterial();
        if (mat != null && targetRenderer != null)
        {
            targetRenderer.material = mat;
        }
    }

    public void Release()
    {
        // detach and resume wandering
        if (leader != null)
        {
            leader.RemoveFollower(this as MonoBehaviour);
            leader = null;
        }
        // do NOT change transform parent; keep agent at scene root to avoid NavMesh issues
        if (agent.isOnNavMesh)
            agent.isStopped = false;
        // reset roam center to current position
        roamCenter = transform.position;
        // restore base speed
        if (agent != null) agent.speed = moveSpeed;

        // restore original material
        if (targetRenderer != null && originalMaterial != null)
            targetRenderer.material = originalMaterial;
    }

    // utility used by leaders when adding follower externally
    public void ApplyMaterialToRenderer(Material m)
    {
        if (targetRenderer != null && m != null)
            targetRenderer.material = m;
    }
}
using System.Collections;
using UnityEngine;
using UnityEngine.AI;

public enum NPCState
{
    Patrol,
    Chase,
    Search
}

[RequireComponent(typeof(NavMeshAgent))]
public class NPCController : MonoBehaviour
{
    [Header("Core")]
    public Transform player;
    public Transform[] waypoints;

    [Header("Movement Tuning")]
    public float patrolSpeed = 2.5f;
    public float chaseSpeed = 4.5f;
    public float acceleration = 8f;
    public float angularSpeed = 120f;
    public float stoppingDistance = 0.5f;

    [Header("Patrol Behavior")]
    public bool loop = true;
    public bool pingPong = false;
    public float waitAtWaypointSeconds = 1.5f;
    public bool randomizeWaypoints = false;

    [Header("Detection")]
    public float detectionRadius = 10f;
    public float loseSightSeconds = 3f; // legacy, not used for giving up
    public float attackRange = 1.5f;
    public LayerMask losObstacles = ~0; // obstacles to block LOS
    public Transform eyePoint; // optional custom origin for LOS ray
    public float fieldOfViewAngle = 120f; // optional FOV cone

    [Header("Animation (optional)")]
    public Animator animator;
    public string runningBool = "Running";

    [Header("Reset Logic")]
    // Simple reset countdown while searching; when it hits 0, return to patrol
    public float searchResetSeconds = 3f;

    private NavMeshAgent agent;
    private NPCState state = NPCState.Patrol;
    private int currentWaypointIndex = 0;
    private int direction = 1; // used for ping-pong
    private Coroutine waitRoutine;
    private float pathRecalcCooldown = 0.5f;
    private float lastPathRequestTime = -999f;

    // Track the last known player position for rerouting/search
    private bool hasLastKnownPlayerPos = false;
    private Vector3 lastKnownPlayerPos;

    // countdown timer used in Search state
    private float searchTimer = 0f;

    void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        if (animator == null) animator = GetComponent<Animator>();

        // tune agent basic settings
        agent.speed = patrolSpeed;
        agent.acceleration = acceleration;
        agent.angularSpeed = angularSpeed;
        agent.stoppingDistance = stoppingDistance;
        agent.autoBraking = true;
        agent.autoRepath = true; // allow auto repath when blocked
    }

    void OnEnable()
    {
        // start patrol if we have waypoints
        if (waypoints != null && waypoints.Length > 0)
        {
            currentWaypointIndex = GetNextWaypointIndex(true);
            SetDestinationSafe(waypoints[currentWaypointIndex].position);
        }
    }

    void Update()
    {
        switch (state)
        {
            case NPCState.Patrol:
                UpdatePatrol();
                DetectPlayerAndMaybeChase();
                break;
            case NPCState.Chase:
                UpdateChase();
                break;
            case NPCState.Search:
                UpdateSearch();
                break;
        }

        // Basic animation: running when moving
        if (animator)
        {
            bool moving = agent.hasPath && agent.velocity.sqrMagnitude > 0.01f;
            animator.SetBool(runningBool, moving);
        }
    }

    // PATROL
    private void UpdatePatrol()
    {
        if (waypoints == null || waypoints.Length == 0)
            return;

        // advance when close
        if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance)
        {
            if (waitRoutine == null && waitAtWaypointSeconds > 0f)
            {
                waitRoutine = StartCoroutine(WaitThenGotoNext());
            }
            else if (waitAtWaypointSeconds <= 0f)
            {
                GotoNextWaypoint();
            }
        }

        // handle blocked/invalid path
        HandlePathStatusRecovery(() => SetDestinationSafe(waypoints[currentWaypointIndex].position));
    }

    private IEnumerator WaitThenGotoNext()
    {
        agent.isStopped = true;
        yield return new WaitForSeconds(waitAtWaypointSeconds);
        agent.isStopped = false;
        waitRoutine = null;
        GotoNextWaypoint();
    }

    private void GotoNextWaypoint()
    {
        currentWaypointIndex = GetNextWaypointIndex(false);
        SetDestinationSafe(waypoints[currentWaypointIndex].position);
    }

    private int GetNextWaypointIndex(bool onStart)
    {
        if (randomizeWaypoints && !onStart)
        {
            int next;
            if (waypoints.Length <= 1) return 0;
            do { next = Random.Range(0, waypoints.Length); } while (next == currentWaypointIndex);
            return next;
        }

        if (pingPong && waypoints.Length > 1)
        {
            int next = currentWaypointIndex + direction;
            if (next >= waypoints.Length || next < 0)
            {
                direction *= -1;
                next = Mathf.Clamp(currentWaypointIndex + direction, 0, waypoints.Length - 1);
            }
            return next;
        }

        // default loop
        int idx = onStart ? currentWaypointIndex : currentWaypointIndex + 1;
        if (idx >= waypoints.Length) idx = loop ? 0 : waypoints.Length - 1;
        return idx;
    }

    // DETECTION
    private void DetectPlayerAndMaybeChase()
    {
        if (player == null) return;

        Vector3 origin = eyePoint ? eyePoint.position : transform.position + Vector3.up * 1.5f;
        Vector3 toPlayer = player.position - origin;
        float dist = toPlayer.magnitude;
        if (dist > detectionRadius) return;

        // FOV check
        if (fieldOfViewAngle < 180f)
        {
            float angle = Vector3.Angle(transform.forward, toPlayer);
            if (angle > fieldOfViewAngle * 0.5f) return;
        }

        // LOS raycast
        if (!Physics.Raycast(origin, toPlayer.normalized, out RaycastHit hit, dist, losObstacles))
        {
            // no hit means nothing blocked
            BeginChase();
            return;
        }
        if (hit.transform == player)
        {
            BeginChase();
        }
    }

    private void BeginChase()
    {
        state = NPCState.Chase;
        agent.speed = chaseSpeed;
        // Initialize last known position when beginning chase
        if (player)
        {
            lastKnownPlayerPos = player.position;
            hasLastKnownPlayerPos = true;
        }
    }

    // CHASE
    private void UpdateChase()
    {
        if (player == null)
        {
            ReturnToPatrol();
            return;
        }

        // refresh destination periodically to avoid thrashing
        if (Time.time - lastPathRequestTime > 0.1f)
        {
            SetDestinationSafe(player.position);
            lastPathRequestTime = Time.time;
        }

        bool hasLOS = HasLineOfSight();

        if (hasLOS)
        {
            // Update last known position while player is visible
            lastKnownPlayerPos = player.position;
            hasLastKnownPlayerPos = true;
        }
        else
        {
            // Immediately switch to searching the last known position when LOS is lost
            if (hasLastKnownPlayerPos)
            {
                state = NPCState.Search;
                agent.isStopped = false;
                SetDestinationSafe(lastKnownPlayerPos);
                // start/reset search countdown
                searchTimer = Mathf.Max(0f, searchResetSeconds);
                return;
            }
        }

        // Attack range: stop and "react" (no combat implemented)
        float distToPlayer = Vector3.Distance(transform.position, player.position);
        if (distToPlayer <= attackRange)
        {
            agent.isStopped = true;
            // TODO: trigger an attack animation or reaction
        }
        else
        {
            agent.isStopped = false;
        }

        // handle blocked/invalid path
        HandlePathStatusRecovery(() =>
        {
            // If reroute is needed, try last known player position first
            if (hasLastKnownPlayerPos)
                SetDestinationSafe(lastKnownPlayerPos);
            else
                SetDestinationSafe(player.position);
        });
    }

    private void UpdateSearch()
    {
        // If we regain LOS during search, resume chase
        if (HasLineOfSight())
        {
            BeginChase();
            return;
        }

        // Head to last known player position
        if (hasLastKnownPlayerPos)
        {
            // countdown and reset when it hits 0
            if (searchTimer > 0f)
            {
                searchTimer -= Time.deltaTime;
                if (searchTimer <= 0f)
                {
                    hasLastKnownPlayerPos = false;
                    ReturnToPatrol();
                    return;
                }
            }

            // If close enough to last known position, also give up
            if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance)
            {
                hasLastKnownPlayerPos = false;
                ReturnToPatrol();
                return;
            }

            // If path becomes invalid while searching, retry after cooldown
            HandlePathStatusRecovery(() =>
            {
                SetDestinationSafe(lastKnownPlayerPos);
            });
        }
        else
        {
            ReturnToPatrol();
        }
    }

    private void ReturnToPatrol()
    {
        state = NPCState.Patrol;
        agent.speed = patrolSpeed;
        agent.isStopped = false;
        // Reset search timer on patrol
        searchTimer = 0f;
        if (waypoints != null && waypoints.Length > 0)
        {
            SetDestinationSafe(waypoints[currentWaypointIndex].position);
        }
    }

    // Helpers
    private bool HasLineOfSight()
    {
        if (player == null) return false;
        Vector3 origin = eyePoint ? eyePoint.position : transform.position + Vector3.up * 1.5f;
        Vector3 toPlayer = player.position - origin;
        float dist = toPlayer.magnitude;
        return Physics.Raycast(origin, toPlayer.normalized, out RaycastHit hit, dist, losObstacles) && hit.transform == player;
    }

    private void HandlePathStatusRecovery(System.Action retryAction)
    {
        // Detect partial/invalid path and retry after a brief cooldown
        if (agent.pathStatus == NavMeshPathStatus.PathInvalid || agent.pathStatus == NavMeshPathStatus.PathPartial)
        {
            // Debug the reroute condition once per cooldown
            if (Time.time - lastPathRequestTime > pathRecalcCooldown)
            {
                Debug.Log($"[NPCController] Path {agent.pathStatus} detected. Rerouting from {transform.position} to new target.");
                lastPathRequestTime = Time.time;
                retryAction?.Invoke();
            }
        }
    }

    private void SetDestinationSafe(Vector3 pos)
    {
        if (!agent.isOnNavMesh) return;
        agent.SetDestination(pos);
    }


    public void PlayStep()
    {

    }
}

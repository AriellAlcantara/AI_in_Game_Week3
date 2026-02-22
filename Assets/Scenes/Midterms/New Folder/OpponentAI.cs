using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public class OpponentAI : MonoBehaviour, ILeader
{
    [Header("Core")]
    public string wandererTag = "Wanderer";
    public string playerTag = "Player";

    [Header("Tuning")]
    public float detectionRadius = 15f;
    public float collectRange = 1.2f;
    public float repathInterval = 0.2f;

    [Header("Behavior")]
    public float chaseMultiplier = 1.5f; // if opponentFollowers > targetFollowers * chaseMultiplier -> chase target
    public float fleeMultiplier = 0.9f;  // if opponentFollowers < targetFollowers * fleeMultiplier -> flee
    public float fleeDistance = 8f;

    [Header("Visuals")]
    // Optional material to apply to followers when claimed
    public Material leaderMaterial;

    [Header("Animation (optional)")]
    public Animator animator;
    public string runningBool = "Running";

    private NavMeshAgent agent;
    private Transform player;
    private float lastRepath = -999f;

    private List<WandererUnit> followers = new List<WandererUnit>();

    void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        var go = GameObject.FindGameObjectWithTag(playerTag);
        if (go) player = go.transform;
    }

    void Update()
    {
        // Choose the nearest leader target (could be player or another opponent)
        var targetLeaderMb = FindNearestLeader();

        int myCount = GetFollowerCount();
        int targetCount = 0;
        if (targetLeaderMb != null)
        {
            var leaderIface = targetLeaderMb as ILeader;
            if (leaderIface != null)
                targetCount = leaderIface.GetFollowerCount();
        }

        bool shouldChase = targetLeaderMb != null && (myCount > targetCount * chaseMultiplier);
        bool shouldFlee = targetLeaderMb != null && (myCount < targetCount * fleeMultiplier);

        if (shouldChase && targetLeaderMb != null)
        {
            ChaseTarget(targetLeaderMb.transform);
        }
        else if (shouldFlee && targetLeaderMb != null)
        {
            FleeFromTarget(targetLeaderMb.transform);
        }
        else
        {
            CollectNearbyWanderer();
        }

        if (animator)
        {
            bool moving = agent.hasPath && agent.velocity.sqrMagnitude > 0.01f;
            animator.SetBool(runningBool, moving);
        }
    }

    // Find nearest leader (player or other OpponentAI), excluding self
    private MonoBehaviour FindNearestLeader()
    {
        float bestDist = float.MaxValue;
        MonoBehaviour best = null;

        // player(s)
        var players = FindObjectsOfType<PlayerFollowers>();
        foreach (var p in players)
        {
            var mb = p as MonoBehaviour;
            if (mb == null) continue;
            float d = (mb.transform.position - transform.position).sqrMagnitude;
            if (d < bestDist)
            {
                bestDist = d;
                best = mb;
            }
        }

        // other opponents
        var others = FindObjectsOfType<OpponentAI>();
        foreach (var o in others)
        {
            if (o == this) continue;
            var mb = o as MonoBehaviour;
            float d = (mb.transform.position - transform.position).sqrMagnitude;
            if (d < bestDist)
            {
                bestDist = d;
                best = mb;
            }
        }

        return best;
    }

    private void ChaseTarget(Transform target)
    {
        if (target == null) return;
        if (Time.time - lastRepath < repathInterval) return;
        lastRepath = Time.time;
        if (!agent.isOnNavMesh) return;
        agent.isStopped = false;
        agent.SetDestination(target.position);
    }

    private void FleeFromTarget(Transform target)
    {
        if (target == null) return;
        if (!agent.isOnNavMesh) return;
        Vector3 dir = (transform.position - target.position).normalized;
        Vector3 dest = transform.position + dir * fleeDistance;
        NavMeshHit hit;
        if (NavMesh.SamplePosition(dest, out hit, 4f, NavMesh.AllAreas))
        {
            agent.isStopped = false;
            agent.SetDestination(hit.position);
        }
    }

    private void CollectNearbyWanderer()
    {
        if (Time.time - lastRepath < repathInterval) return;
        lastRepath = Time.time;

        if (!agent.isOnNavMesh) return;

        // find closest unclaimed wanderer
        WandererUnit best = null;
        float bestDist = float.MaxValue;
        var all = GameObject.FindGameObjectsWithTag(wandererTag);
        foreach (var go in all)
        {
            var unit = go.GetComponent<WandererUnit>();
            if (unit == null) continue;
            if (unit.HasLeader()) continue;

            float d = (go.transform.position - transform.position).sqrMagnitude;
            if (d < bestDist)
            {
                bestDist = d;
                best = unit;
            }
        }

        if (best != null)
        {
            agent.isStopped = false;
            agent.SetDestination(best.transform.position);

            // if close enough, collect
            if (bestDist <= collectRange * collectRange)
            {
                Collect(best);
            }
        }
        else
        {
            // idle roam: stand or small random walk (not implemented)
            agent.ResetPath();
        }
    }

    // Existing concrete Add/Remove for WandererUnit
    public void AddFollower(WandererUnit unit)
    {
        if (unit == null) return;
        if (!followers.Contains(unit))
            followers.Add(unit);

        // apply leader material if available
        if (leaderMaterial != null)
            unit.ApplyMaterialToRenderer(leaderMaterial);
    }

    public void RemoveFollower(WandererUnit unit)
    {
        if (unit == null) return;
        followers.Remove(unit);
    }

    // ILeader interface wrappers using MonoBehaviour to satisfy interface
    public void AddFollower(MonoBehaviour unit)
    {
        AddFollower(unit as WandererUnit);
    }

    public void RemoveFollower(MonoBehaviour unit)
    {
        RemoveFollower(unit as WandererUnit);
    }

    // Backwards-compatible wrapper
    public void Collect(WandererUnit unit)
    {
        if (unit == null) return;
        if (unit.HasLeader()) return;
        unit.SetLeader(this);
        // unit.SetLeader calls AddFollower via WandererUnit
    }

    public int GetFollowerCount()
    {
        return followers.Count;
    }

    // Return leader material for ILeader
    public Material GetLeaderMaterial()
    {
        return leaderMaterial;
    }

    void OnTriggerEnter(Collider other)
    {
        // collision with other leaders
        var otherLeader = other.GetComponent<OpponentAI>();
        if (otherLeader != null && otherLeader != this)
        {
            ResolveLeaderCollision(otherLeader);
            return;
        }

        // collision with player
        if (other.CompareTag(playerTag))
        {
            var pf = other.GetComponent<PlayerFollowers>();
            int playerCount = pf ? pf.GetFollowerCount() : 0;
            if (GetFollowerCount() < playerCount)
            {
                // we lose
                LoseLeader();
            }
            else if (GetFollowerCount() > playerCount)
            {
                // player loses: notify and destroy player gameobject
                if (pf != null)
                {
                    pf.OnDefeatedBy(this as MonoBehaviour);
                    Debug.Log($"Opponent '{name}' destroyed the player '{pf.gameObject.name}'.");
                    Destroy(pf.gameObject);
                }
                else
                    Debug.Log("Player has no PlayerFollowers component to handle defeat.");
            }
        }
    }

    private void ResolveLeaderCollision(OpponentAI other)
    {
        if (other == null) return;
        if (GetFollowerCount() < other.GetFollowerCount())
        {
            LoseLeader();
        }
        else if (GetFollowerCount() > other.GetFollowerCount())
        {
            other.LoseLeader();
        }
        else
        {
            // tie -> no immediate destruction
            Debug.Log("Leader collision: tie, no destruction.");
        }
    }

    private void LoseLeader()
    {
        // release followers
        foreach (var f in new List<WandererUnit>(followers))
        {
            if (f != null)
            {
                f.Release();
            }
        }
        followers.Clear();

        Debug.Log($"Opponent '{name}' destroyed due to having fewer followers.");
        Destroy(gameObject);
    }
}

using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class PlayerFollowers : MonoBehaviour, ILeader
{
    private List<WandererUnit> followers = new List<WandererUnit>();

    [Header("Visuals")]
    public Material leaderMaterial; // optional material for player-led followers

    public void AddFollower(WandererUnit unit)
    {
        if (unit == null) return;
        if (!followers.Contains(unit))
            followers.Add(unit);

        // apply material if available
        if (leaderMaterial != null)
            unit.ApplyMaterialToRenderer(leaderMaterial);
    }

    // ILeader-compatible wrapper
    public void AddFollower(MonoBehaviour unit)
    {
        AddFollower(unit as WandererUnit);
    }

    public void RemoveFollower(WandererUnit unit)
    {
        if (unit == null) return;
        followers.Remove(unit);
    }

    // ILeader-compatible wrapper
    public void RemoveFollower(MonoBehaviour unit)
    {
        RemoveFollower(unit as WandererUnit);
    }

    public int GetFollowerCount()
    {
        return followers.Count;
    }

    // Called when an opponent defeats the player
    public void OnDefeatedBy(MonoBehaviour opponent)
    {
        // release all followers
        foreach (var f in new List<WandererUnit>(followers))
        {
            if (f != null)
            {
                f.Release();
            }
        }
        followers.Clear();

        Debug.Log($"Player defeated by {opponent?.name ?? "Unknown"}. All followers released.");
    }

    // ILeader material provider
    public Material GetLeaderMaterial()
    {
        return leaderMaterial;
    }
}
using UnityEngine;

public interface ILeader
{
    void AddFollower(MonoBehaviour unit);
    void RemoveFollower(MonoBehaviour unit);
    int GetFollowerCount();

    // Return a material that followers should use when claimed (can be null)
    Material GetLeaderMaterial();
}
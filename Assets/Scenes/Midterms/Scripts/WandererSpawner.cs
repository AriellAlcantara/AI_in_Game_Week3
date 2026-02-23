using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Spawns a number of wanderer prefabs around this GameObject on the NavMesh.
/// Assign a prefab that contains a NavMeshAgent and a WandererUnit component and set its tag to "Wanderer" in Inspector.
/// </summary>
public class WandererSpawner : MonoBehaviour
{
    [Header("Prefab")]
    public GameObject wandererPrefab;

    [Header("Spawn")]
    public int spawnCount = 50;
    public float spawnRadius = 20f;
    public float maxSampleDistance = 2f;
    public bool spawnOnStart = true;

    [Header("Parenting")]
    public Transform parentContainer; // optional parent for spawned objects

    // When sampling fails, try these extra radii (meters)
    private readonly float[] fallbackSampleMultipliers = new float[] { 2f, 4f, 8f };

    void Start()
    {
        if (spawnOnStart)
            SpawnAll();
    }

    [ContextMenu("Spawn All Wanderers")]
    public void SpawnAll()
    {
        if (wandererPrefab == null)
        {
            Debug.LogError("WandererSpawner: wandererPrefab is not assigned.");
            return;
        }

        if (parentContainer == null)
            parentContainer = this.transform;

        // Quick check: is there any NavMesh near the spawner at all?
        NavMeshHit centerHit;
        bool hasNav = NavMesh.SamplePosition(transform.position, out centerHit, Mathf.Max(maxSampleDistance, 1f), NavMesh.AllAreas);
        if (!hasNav)
        {
            Debug.LogWarning("WandererSpawner: No NavMesh found near spawner position. Increase maxSampleDistance or move spawner to a NavMesh surface.");
            // continue, attempts will still be made for each spawn
        }

        int spawned = 0;
        for (int i = 0; i < spawnCount; i++)
        {
            // random point in circle
            Vector2 r = Random.insideUnitCircle * spawnRadius;
            Vector3 pos = transform.position + new Vector3(r.x, 0f, r.y);

            // sample NavMesh to get a valid position
            NavMeshHit hit;
            bool placed = false;

            // primary attempt
            if (NavMesh.SamplePosition(pos, out hit, maxSampleDistance, NavMesh.AllAreas))
            {
                InstantiateAt(hit.position);
                spawned++;
                placed = true;
            }
            else
            {
                // fallback attempts: try random nearby samples and increasing radii
                for (int attempt = 0; attempt < fallbackSampleMultipliers.Length && !placed; attempt++)
                {
                    float sampleRadius = maxSampleDistance * fallbackSampleMultipliers[attempt];
                    // try a few random offsets within the spawnRadius to find NavMesh
                    for (int j = 0; j < 4 && !placed; j++)
                    {
                        Vector2 r2 = Random.insideUnitCircle * spawnRadius;
                        Vector3 p2 = transform.position + new Vector3(r2.x, 0f, r2.y);
                        if (NavMesh.SamplePosition(p2, out hit, sampleRadius, NavMesh.AllAreas))
                        {
                            InstantiateAt(hit.position);
                            spawned++;
                            placed = true;
                            break;
                        }
                    }
                }

                if (!placed)
                {
                    // final brute-force: try radial samples around original pos
                    int radialSteps = 12;
                    for (int step = 1; step <= 5 && !placed; step++)
                    {
                        float angleStep = 360f / radialSteps;
                        float radiusStep = step * (maxSampleDistance + 0.5f);
                        for (int a = 0; a < radialSteps; a++)
                        {
                            float ang = a * angleStep * Mathf.Deg2Rad;
                            Vector3 p3 = pos + new Vector3(Mathf.Cos(ang), 0f, Mathf.Sin(ang)) * radiusStep;
                            if (NavMesh.SamplePosition(p3, out hit, maxSampleDistance * 1.5f, NavMesh.AllAreas))
                            {
                                InstantiateAt(hit.position);
                                spawned++;
                                placed = true;
                                break;
                            }
                        }
                    }
                }

                if (!placed)
                {
                    Debug.LogWarning("WandererSpawner: Failed to sample NavMesh for spawn position.");
                }
            }
        }

        Debug.Log($"WandererSpawner: Spawned {spawned}/{spawnCount} wanderers under '{parentContainer.name}'.");
    }

    private void InstantiateAt(Vector3 position)
    {
        GameObject go = Instantiate(wandererPrefab, position, Quaternion.identity, parentContainer);
        // ensure tag if possible
        try { go.tag = "Wanderer"; } catch { }
    }
}

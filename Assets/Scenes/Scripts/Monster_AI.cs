using UnityEngine;
using UnityEngine.AI;

public class Monster_AI : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created


    private NavMeshAgent agent;
    public Transform ellenPos;
    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
    }

    // Update is called once per frame
    void Update()
    {
        agent.SetDestination(ellenPos.position);
    }
}

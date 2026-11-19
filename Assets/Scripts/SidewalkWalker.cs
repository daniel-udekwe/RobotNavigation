using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// One pedestrian walking along a SidewalkCrowd path. Added automatically by SidewalkCrowd.
/// Uses NavMeshAgent so people avoid each other and the robot's planner agent.
/// </summary>
[RequireComponent(typeof(NavMeshAgent))]
public class SidewalkWalker : MonoBehaviour
{
    public bool forward;            // true = A->B, false = B->A
    public float lateral;           // metres to the walker's right of the centreline

    SidewalkCrowd crowd;
    NavMeshAgent agent;
    Animator anim;
    int next;                       // index of the waypoint we're heading to
    float respawnAt = -1f;

    public void Init(SidewalkCrowd c, bool goingForward, bool scatterAlongPath)
    {
        crowd = c;
        forward = goingForward;
        agent = GetComponent<NavMeshAgent>();
        anim = GetComponentInChildren<Animator>();
        agent.autoBraking = false;
        BeginWalk(scatterAlongPath);
    }

    void BeginWalk(bool scatter)
    {
        respawnAt = -1f;
        lateral = crowd.RandomLateral();
        agent.speed = crowd.RandomSpeed();

        Vector3 start;
        if (scatter)
        {
            // Start somewhere along the sidewalk so it is populated from frame one.
            int k = Random.Range(0, crowd.Count - 1);
            float t = Random.value;
            start = crowd.PointOnSegment(k, t, lateral, forward);
            next = forward ? k + 1 : k;
        }
        else
        {
            int first = forward ? 0 : crowd.Count - 1;
            start = crowd.PointAt(first, lateral, forward);
            next = forward ? 1 : crowd.Count - 2;
        }

        if (NavMesh.SamplePosition(start, out var hit, 2f, new NavMeshQueryFilter
            { agentTypeID = agent.agentTypeID, areaMask = NavMesh.AllAreas }))
        {
            agent.Warp(hit.position);
            transform.rotation = Quaternion.LookRotation(crowd.TravelDir(Mathf.Clamp(next, 0, crowd.Count - 1), forward));
            SetTarget();
        }
        else
        {
            Debug.LogWarning($"{name}: no Humanoid NavMesh near {start}. Bake a Humanoid NavMeshSurface on the ground.");
        }
    }

    void SetTarget()
    {
        if (agent.isOnNavMesh)
            agent.SetDestination(crowd.PointAt(next, lateral, forward));
    }

    void Update()
    {
        if (crowd == null || agent == null) return;
        if (anim != null) anim.SetFloat("Speed", agent.velocity.magnitude);

        // Waiting off-stage before re-entering
        if (respawnAt > 0f)
        {
            if (Time.time >= respawnAt) { SetVisible(true); BeginWalk(false); }
            return;
        }

        if (!agent.enabled || !agent.isOnNavMesh || agent.pathPending) return;

        if (agent.remainingDistance < 0.6f)
        {
            bool atEnd = forward ? next >= crowd.Count - 1 : next <= 0;
            if (atEnd)
            {
                // Reached the end of the sidewalk: vanish, then re-enter at the start.
                SetVisible(false);
                agent.ResetPath();
                respawnAt = Time.time + Mathf.Max(0.01f, crowd.RandomRespawnDelay());
                return;
            }
            next += forward ? 1 : -1;
            // Drift a little sideways now and then, like real people do.
            lateral = Mathf.Lerp(lateral, crowd.RandomLateral(), 0.3f);
            SetTarget();
        }
    }

    void SetVisible(bool on)
    {
        foreach (var r in GetComponentsInChildren<Renderer>()) r.enabled = on;
        foreach (var c in GetComponentsInChildren<Collider>()) c.enabled = on;
        // While hidden, switch the agent off so nobody (people or robot) avoids an invisible person.
        agent.enabled = on;
    }
}

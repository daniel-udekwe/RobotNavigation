using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Spawns pedestrians who walk along a sidewalk in both directions, forever.
///
/// Setup:
///  1. Create empty GameObjects as waypoints along the CENTRE of the sidewalk, in order
///     from one end (A) to the other (B). 2 points for a straight sidewalk, more for bends.
///  2. Add this component to an empty "SidewalkCrowd" object and drag the waypoints
///     into the Waypoints list in order.
///  3. Assign a pedestrian prefab (needs a NavMeshAgent, Agent Type = Humanoid).
///  4. Bake a Humanoid NavMesh on the ground (second NavMeshSurface).
/// People heading A->B are "same direction" (set your robot's goal toward B),
/// people heading B->A are "opposite direction".
/// </summary>
public class SidewalkCrowd : MonoBehaviour
{
    [Header("Sidewalk")]
    public Transform[] waypoints;               // centreline, ordered A -> B
    public float sidewalkWidth = 1.5f;          // m
    public float edgeMargin = 0.3f;             // keep people this far from edges

    [Header("People")]
    public GameObject pedestrianPrefab;
    [Tooltip("Optional: several different people. If filled, one is picked at random for each spawn.")]
    public GameObject[] pedestrianPrefabs;
    public int walkingAtoB = 4;                 // same direction as the robot (if robot goes to B)
    public int walkingBtoA = 4;                 // opposite direction
    public Vector2 speedRange = new Vector2(1.0f, 1.5f);   // m/s
    [Range(0f, 1f)] public float keepRightBias = 0.5f;     // 0 = anywhere, 1 = always right half
    public Vector2 respawnDelay = new Vector2(0f, 4f);     // s, before re-entering at the start
    [Header("Auto-setup for raw character models (e.g. Mixamo .fbx)")]
    [Tooltip("Assigned to any spawned person whose Animator has no controller (fixes the T-pose).")]
    public RuntimeAnimatorController pedestrianAnimator;
    [Tooltip("Layer given to spawned people so the robot's bumper sees them.")]
    public string peopleLayer = "People";
    public float personRadius = 0.3f;
    public float personHeight = 1.7f;
    [Tooltip("Random colours on spawned capsules. Turn OFF for real character models.")]
    public bool randomColours = true;

    void Start()
    {
        bool hasPrefab = pedestrianPrefab != null || (pedestrianPrefabs != null && pedestrianPrefabs.Length > 0);
        if (waypoints == null || waypoints.Length < 2 || !hasPrefab)
        {
            Debug.LogError("SidewalkCrowd: needs at least 2 waypoints and a pedestrian prefab.");
            enabled = false;
            return;
        }
        for (int i = 0; i < walkingAtoB; i++) Spawn(true, i);
        for (int i = 0; i < walkingBtoA; i++) Spawn(false, i);
    }

    void Spawn(bool forward, int index)
    {
        GameObject prefab = pedestrianPrefab;
        if (pedestrianPrefabs != null && pedestrianPrefabs.Length > 0)
        {
            var pick = pedestrianPrefabs[Random.Range(0, pedestrianPrefabs.Length)];
            if (pick != null) prefab = pick;
        }
        var go = Instantiate(prefab, transform);
        go.name = $"Pedestrian {(forward ? "A>B" : "B>A")} {index}";
        if (randomColours)
            foreach (var r in go.GetComponentsInChildren<Renderer>())
                r.material.color = Random.ColorHSV(0f, 1f, 0.4f, 0.8f, 0.5f, 0.9f);

        AutoSetup(go);

        var walker = go.GetComponent<SidewalkWalker>();
        if (walker == null) walker = go.AddComponent<SidewalkWalker>();
        walker.Init(this, forward, scatterAlongPath: true);
    }

    /// Makes a raw character model usable as a pedestrian: animator, agent, collider, layer.
    void AutoSetup(GameObject go)
    {
        // Animator: give it the walk/idle controller, and let the NavMeshAgent do the moving.
        foreach (var anim in go.GetComponentsInChildren<Animator>())
        {
            if (anim.runtimeAnimatorController == null && pedestrianAnimator != null)
                anim.runtimeAnimatorController = pedestrianAnimator;
            anim.applyRootMotion = false;
        }
        if (pedestrianAnimator == null && go.GetComponentInChildren<Animator>() is Animator a && a.runtimeAnimatorController == null)
            Debug.LogWarning($"{go.name}: Animator has no controller - assign Pedestrian Animator on SidewalkCrowd.");

        // NavMeshAgent with person-sized settings (only if the prefab didn't bring one).
        if (!go.TryGetComponent<NavMeshAgent>(out var agent))
        {
            agent = go.AddComponent<NavMeshAgent>();
            agent.agentTypeID = HumanoidAgentTypeID();
            agent.radius = personRadius;
            agent.height = personHeight;
            agent.baseOffset = 0f;
            agent.angularSpeed = 360f;
            agent.acceleration = 4f;
            agent.stoppingDistance = 0f;
            agent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
            agent.avoidancePriority = 30;
        }

        // Collider so the robot's safety bumper can detect the person.
        if (go.GetComponentInChildren<Collider>() == null)
        {
            var col = go.AddComponent<CapsuleCollider>();
            col.center = new Vector3(0f, personHeight * 0.5f, 0f);
            col.radius = personRadius * 0.8f;
            col.height = personHeight;
        }

        // Layer
        int layer = LayerMask.NameToLayer(peopleLayer);
        if (layer >= 0)
            foreach (var t in go.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = layer;
    }

    static int HumanoidAgentTypeID()
    {
        for (int i = 0; i < NavMesh.GetSettingsCount(); i++)
        {
            var st = NavMesh.GetSettingsByIndex(i);
            if (NavMesh.GetSettingsNameFromID(st.agentTypeID) == "Humanoid") return st.agentTypeID;
        }
        return 0; // Humanoid is normally id 0
    }

    // ---------- helpers used by SidewalkWalker ----------

    public int Count => waypoints.Length;

    public float RandomSpeed() => Random.Range(speedRange.x, speedRange.y);

    public float RandomRespawnDelay() => Random.Range(respawnDelay.x, respawnDelay.y);

    /// Lateral offset in metres, positive = walker's right-hand side.
    public float RandomLateral()
    {
        float half = Mathf.Max(0f, sidewalkWidth * 0.5f - edgeMargin);
        float anywhere = Random.Range(-1f, 1f);
        float rightHalf = Random.Range(0f, 1f);
        return half * Mathf.Lerp(anywhere, rightHalf, keepRightBias);
    }

    /// Direction of travel at waypoint i (flattened), for a walker going forward (A->B) or back.
    public Vector3 TravelDir(int i, bool forward)
    {
        int a = Mathf.Max(i - 1, 0), b = Mathf.Min(i + 1, Count - 1);
        Vector3 d = waypoints[b].position - waypoints[a].position; d.y = 0;
        d.Normalize();
        return forward ? d : -d;
    }

    /// World point near waypoint i, shifted sideways relative to the walker's direction.
    public Vector3 PointAt(int i, float lateral, bool forward)
    {
        Vector3 right = Vector3.Cross(Vector3.up, TravelDir(i, forward));
        return waypoints[i].position + right * lateral;
    }

    public Vector3 PointOnSegment(int k, float t, float lateral, bool forward)
    {
        Vector3 p = Vector3.Lerp(waypoints[k].position, waypoints[k + 1].position, t);
        Vector3 d = waypoints[k + 1].position - waypoints[k].position; d.y = 0; d.Normalize();
        if (!forward) d = -d;
        return p + Vector3.Cross(Vector3.up, d) * lateral;
    }

    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Length < 2) return;
        for (int i = 0; i < waypoints.Length - 1; i++)
        {
            if (!waypoints[i] || !waypoints[i + 1]) continue;
            Vector3 a = waypoints[i].position, b = waypoints[i + 1].position;
            Vector3 d = b - a; d.y = 0; d.Normalize();
            Vector3 r = Vector3.Cross(Vector3.up, d) * sidewalkWidth * 0.5f;
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(a, b);
            Gizmos.color = new Color(1f, 0.6f, 0f);
            Gizmos.DrawLine(a + r, b + r);
            Gizmos.DrawLine(a - r, b - r);
        }
        Gizmos.color = Color.green; if (waypoints[0]) Gizmos.DrawWireSphere(waypoints[0].position, 0.3f);
        Gizmos.color = Color.red;   if (waypoints[^1]) Gizmos.DrawWireSphere(waypoints[^1].position, 0.3f);
    }
}

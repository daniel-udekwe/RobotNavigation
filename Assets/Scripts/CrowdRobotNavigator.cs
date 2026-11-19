using UnityEngine;
using UnityEngine.AI;
using Unity.AI.Navigation;

/// <summary>
/// Drives an imported TurtleBot3 Burger (ArticulationBody wheels) through a crowd.
///
/// How it works:
///  - A separate "planner" NavMeshAgent plans the route AND does local avoidance
///    of the pedestrians (who are NavMeshAgents too). It never moves the robot itself.
///  - Each physics step the planner is snapped to the robot's real position, and its
///    desired velocity is turned into left/right wheel speeds (differential drive).
///  - A forward sphere-cast acts as a safety bumper: if anything is too close in front,
///    the robot stops driving forward (it may still turn).
///
/// Setup:
///  1. Create an empty GameObject "RobotPlanner" (NOT a child of the robot) and add a
///     NavMeshAgent: Agent Type = your TurtleBot type, Radius 0.12, Speed 0.22,
///     Acceleration 1, Stopping Distance 0.1, Obstacle Avoidance Quality = High,
///     Priority 90 (higher number = yields to others).
///  2. Put this script on the robot root; assign Planner, both wheels and Robot Base.
///  3. Put pedestrians on a "People" layer and select it in Bumper Layers.
///  4. Play, then click the ground in the Game view to set a goal.
/// </summary>
public class CrowdRobotNavigator : MonoBehaviour
{
    [Header("References")]
    public NavMeshAgent planner;
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;
    [Tooltip("The robot body to read pose from (e.g. base_footprint). Empty = this transform.")]
    public Transform robotBase;
    public bool invertLeft = false;
    public bool invertRight = false;

    [Header("TurtleBot3 Burger specs")]
    public float wheelRadius = 0.033f;      // m
    public float wheelSeparation = 0.160f;  // m
    public float maxLinearSpeed = 0.22f;    // m/s
    public float maxAngularSpeed = 2.84f;   // rad/s
    public float headingGain = 2.5f;

    [Header("Goal")]
    public Transform target;                // optional
    public bool clickToSetGoal = true;
    [Tooltip("Layers you can click on to set a goal (your ground/collision mesh).")]
    public LayerMask clickLayers = ~0;

    [Header("Safety bumper")]
    public LayerMask bumperLayers = ~0;
    public float bumperRadius = 0.10f;
    public float stopDistance = 0.35f;      // m in front of the robot
    public float bumperHeight = 0.10f;

    public enum DriveMode { WheelPhysics, Kinematic }
    [Header("Wheel drive")]
    [Tooltip("WheelPhysics: wheels push the robot via friction (realistic, sensitive to setup).\n" +
             "Kinematic: robot body is moved directly at the commanded speed; wheels spin for looks.")]
    public DriveMode driveMode = DriveMode.Kinematic;
    public float driveDamping = 1000f;
    public float driveForceLimit = 100f;

    [Header("Debug")]
    public bool showDebugText = true;

    public bool BlockedAhead { get; private set; }
    string status = "starting";
    float lastV, lastW;
    int clicks, clickHits;

    void Start()
    {
        if (robotBase == null) robotBase = transform;
        if (planner == null) { Debug.LogError("CrowdRobotNavigator: assign a planner NavMeshAgent."); enabled = false; return; }

        planner.updatePosition = false;   // we move the robot with physics, not the agent
        planner.updateRotation = false;
        planner.speed = maxLinearSpeed;
        EnsureOnNavMesh(true);

        SetupDrive(leftWheel);
        SetupDrive(rightWheel);
        if (target != null && planner.isOnNavMesh) planner.SetDestination(target.position);
    }

    /// Snap the planner onto the NavMesh under the robot. Returns true if it is on the NavMesh.
    bool EnsureOnNavMesh(bool log = false)
    {
        if (planner.isOnNavMesh) return true;
        var filter = new NavMeshQueryFilter { agentTypeID = planner.agentTypeID, areaMask = NavMesh.AllAreas };
        if (NavMesh.SamplePosition(robotBase.position, out var hit, 2f, filter))
        {
            planner.Warp(hit.position);
            return planner.isOnNavMesh;
        }
        if (log)
        {
            var sb = new System.Text.StringBuilder("CrowdRobotNavigator diagnostics:\n");
            sb.Append($"  Planner '{planner.name}' agent type: '{NavMesh.GetSettingsNameFromID(planner.agentTypeID)}' (id {planner.agentTypeID})\n");
            sb.Append($"  Robot Base '{robotBase.name}' at {robotBase.position}\n");
            foreach (var s in NavMeshSurface.activeSurfaces)
                sb.Append($"  Surface on '{s.name}': type '{NavMesh.GetSettingsNameFromID(s.agentTypeID)}' (id {s.agentTypeID}), " +
                          $"data {(s.navMeshData ? "loaded" : "MISSING")}, bounds {(s.navMeshData ? s.navMeshData.sourceBounds.ToString() : "-")}\n");
            if (NavMeshSurface.activeSurfaces.Count == 0) sb.Append("  No active NavMeshSurface in the scene.\n");
            for (int i = 0; i < NavMesh.GetSettingsCount(); i++)
            {
                var st = NavMesh.GetSettingsByIndex(i);
                sb.Append($"  Agent type defined: '{NavMesh.GetSettingsNameFromID(st.agentTypeID)}' (id {st.agentTypeID})\n");
            }
            Debug.Log(sb.ToString());
            Debug.LogWarning($"CrowdRobotNavigator: no NavMesh for agent type " +
                $"'{NavMesh.GetSettingsNameFromID(planner.agentTypeID)}' within 2 m of the robot at {robotBase.position}. " +
                "Check the planner's Agent Type matches the baked NavMeshSurface, and that the robot is over the baked area.");
        }
        return false;
    }

    void SetupDrive(ArticulationBody wheel)
    {
        if (wheel == null) { Debug.LogError("CrowdRobotNavigator: assign both wheels."); return; }
        var d = wheel.xDrive;
        d.stiffness = 0f;
        d.damping = driveDamping;
        d.forceLimit = driveForceLimit;
        wheel.xDrive = d;
    }

    void Update()
    {
#if ENABLE_INPUT_SYSTEM
        var mouse = UnityEngine.InputSystem.Mouse.current;
        bool clicked = mouse != null && mouse.leftButton.wasPressedThisFrame;
        Vector2 mousePos = mouse != null ? mouse.position.ReadValue() : Vector2.zero;
#else
        bool clicked = Input.GetMouseButtonDown(0);
        Vector2 mousePos = Input.mousePosition;
#endif
        if (clicked) clicks++;
        if (clickToSetGoal && clicked && Camera.main != null)
        {
            var ray = Camera.main.ScreenPointToRay(mousePos);
            bool rayHit = Physics.Raycast(ray, out var hit, 1000f, clickLayers);
            if (rayHit) clickHits++;
            if (rayHit && EnsureOnNavMesh(true))
            {
                target = null;
                planner.SetDestination(hit.point);
            }
        }
        else if (target != null && planner.isOnNavMesh)
        {
            planner.SetDestination(target.position);
        }
    }

    void FixedUpdate()
    {
        if (!EnsureOnNavMesh()) { status = "planner NOT on NavMesh"; Drive(0, 0); return; }

        // Keep the planner glued to where the robot really is.
        planner.nextPosition = robotBase.position;

        if (planner.pathPending) { status = "computing path"; Drive(0, 0); return; }
        if (!planner.hasPath) { status = "no goal yet - click the ground in the Game view"; Drive(0, 0); return; }
        if (planner.remainingDistance <= planner.stoppingDistance) { status = "arrived"; Drive(0, 0); return; }

        Vector3 desired = planner.desiredVelocity; desired.y = 0;
        if (desired.sqrMagnitude < 1e-4f) { status = "path found but desired velocity is zero"; Drive(0, 0); return; }
        status = "driving";

        Vector3 fwd = robotBase.forward; fwd.y = 0;
        float heading = Vector3.SignedAngle(fwd, desired, Vector3.up) * Mathf.Deg2Rad;

        float w = Mathf.Clamp(headingGain * heading, -maxAngularSpeed, maxAngularSpeed);
        float v = Mathf.Abs(heading) > 0.6f ? 0f
                : Mathf.Min(desired.magnitude, maxLinearSpeed) * Mathf.Cos(heading);

        // Safety bumper: never drive forward into something close in front.
        Vector3 origin = robotBase.position + Vector3.up * bumperHeight;
        BlockedAhead = Physics.SphereCast(origin, bumperRadius, fwd.normalized, out _,
                                          stopDistance, bumperLayers, QueryTriggerInteraction.Ignore);
        if (BlockedAhead) v = 0f;

        Drive(v, -w); // positive SignedAngle = target to the right = clockwise yaw
    }

    /// v: forward m/s, w: yaw rate rad/s (counter-clockwise positive).
    ArticulationBody rootBody;

    void Drive(float v, float w)
    {
        lastV = v; lastW = w;
        if (driveMode == DriveMode.Kinematic)
        {
            if (rootBody == null && robotBase != null)
            {
                var chain = robotBase.GetComponentsInParent<ArticulationBody>();
                if (chain.Length > 0) rootBody = chain[chain.Length - 1];
            }
            if (rootBody != null)
            {
                Vector3 f = robotBase.forward; f.y = 0; f.Normalize();
                Vector3 vel = rootBody.velocity;
                rootBody.velocity = f * v + Vector3.up * Mathf.Min(vel.y, 0f); // keep falling/settling
                Vector3 av = rootBody.angularVelocity;
                rootBody.angularVelocity = new Vector3(av.x, -w, av.z);        // Unity +Y yaw = clockwise
            }
        }
        float left  = (v - w * wheelSeparation / 2f) / wheelRadius;
        float right = (v + w * wheelSeparation / 2f) / wheelRadius;
        SetWheel(leftWheel,  left  * (invertLeft  ? -1 : 1));
        SetWheel(rightWheel, right * (invertRight ? -1 : 1));
    }

    void SetWheel(ArticulationBody wheel, float radPerSec)
    {
        if (wheel == null) return;
        var d = wheel.xDrive;
        d.targetVelocity = radPerSec * Mathf.Rad2Deg; // deg/s
        wheel.xDrive = d;
    }

    void OnGUI()
    {
        if (!showDebugText || planner == null) return;
        float wl = leftWheel ? leftWheel.xDrive.targetVelocity : float.NaN;
        float wr = rightWheel ? rightWheel.xDrive.targetVelocity : float.NaN;
        float spd = leftWheel ? leftWheel.jointVelocity.dofCount > 0 ? leftWheel.jointVelocity[0] * Mathf.Rad2Deg : 0 : 0;
        var chain = robotBase ? robotBase.GetComponentsInParent<ArticulationBody>() : null;
        var root = chain != null && chain.Length > 0 ? chain[chain.Length - 1] : null;
        string txt =
            $"Status: {status}\n" +
            $"Clicks: {clicks} (hit ground: {clickHits})\n" +
            $"Planner on NavMesh: {planner.isOnNavMesh}  hasPath: {planner.hasPath}  remaining: {(planner.hasPath ? planner.remainingDistance.ToString("F2") : "-")} m\n" +
            $"Desired vel: {planner.desiredVelocity.magnitude:F2} m/s   Command v: {lastV:F2} m/s  w: {lastW:F2} rad/s\n" +
            $"Wheel targets L/R: {wl:F0} / {wr:F0} deg/s   Left wheel actual: {spd:F0} deg/s\n" +
            $"Blocked ahead: {BlockedAhead}   Robot pos: {(robotBase ? robotBase.position.ToString("F2") : "-")}\n" +
            $"Robot root body: {(root ? root.name + (root.immovable ? "  (IMMOVABLE - untick it!)" : "") : "none found")}";
        GUI.color = Color.black;
        GUI.Label(new Rect(11, 11, 900, 200), txt);
        GUI.color = Color.yellow;
        GUI.Label(new Rect(10, 10, 900, 200), txt);
    }

    void OnDrawGizmos()
    {
        if (planner != null && planner.hasPath)
        {
            Gizmos.color = Color.cyan;
            var c = planner.path.corners;
            for (int i = 0; i < c.Length - 1; i++) Gizmos.DrawLine(c[i], c[i + 1]);
        }
        if (robotBase != null)
        {
            Gizmos.color = BlockedAhead ? Color.red : Color.green;
            Vector3 f = robotBase.forward; f.y = 0;
            Vector3 o = robotBase.position + Vector3.up * bumperHeight;
            Gizmos.DrawWireSphere(o + f.normalized * stopDistance, bumperRadius);
        }
    }
}

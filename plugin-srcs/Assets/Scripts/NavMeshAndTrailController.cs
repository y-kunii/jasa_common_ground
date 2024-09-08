using System.Collections;
using System.Collections.Generic;
using UnityEditor.Timeline.Actions;
using UnityEngine;
using UnityEngine.AI;
using System.Threading.Tasks;
using Unity.VisualScripting;

public class NavMeshAndTrailController : MonoBehaviour
{
    private NavMeshAgent agent;
    private int goalListCount = 0;
    private Transform goalPoint;
    public List<Transform> goalList;
    private Vector3 direction;
    private Vector3 diffVec;
    public bool directionCheckFlag = true;
    public bool navMotionCheckFlag = false;
    public float RotateSpeed = 100f;

    // 回転中しているかどうか
    public bool IsRotating { get; private set; }
    // 角速度[deg/s]
    public float AngularVelocity { get; private set; }
    // 回転軸
    public Vector3 Axis { get; private set; }
    // 前フレームの姿勢
    private Quaternion _prevRotation;


    // Start is called before the first frame update
    void Start()
    {
        //goalList.Add(goal1);
        //goalList.Add(goal2);
        //goalList.Add(goal3);

        agent = GetComponent<NavMeshAgent>();
        goalPoint = goalList[goalListCount];
        direction = (goalPoint.position - this.transform.position).normalized;

        _prevRotation = this.transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        if (goalPoint != null)
        {
            this.transform.LookAt(goalPoint.transform);
            GeneratePassLine();
        }
    }
    public void GeneratePassLine()
    {
        this.transform.position += this.transform.forward * 0.1f;
        float dis = Vector3.Distance(goalPoint.position, this.transform.position);
        if (Mathf.Abs(dis) < 1.0f)
        {
            if (goalList.Count > goalListCount)
            {
                goalListCount++;
                goalPoint = goalList[goalListCount];
            }
            else
            {
                goalPoint = null;
            }
        }
    }
}

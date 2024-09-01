using System.Collections;
using System.Collections.Generic;
using UnityEditor.Timeline.Actions;
using UnityEngine;
using UnityEngine.AI;
using System.Threading.Tasks;

public class NavMeshContoroller : MonoBehaviour
{
    private NavMeshAgent agent;
    private int goalListCount = 0;
    private Transform goalPoint;
    public Transform goal1;
    public Transform goal2;
    public Transform goal3;
    public List<Transform> goalList;
    private Vector3 direction;
    private Vector3 diffVec;
    public bool directionCheckFlag = true;
    public bool navMotionCheckFlag = false;
    public float RotateSpeed = 10f;

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
        goalList.Add(goal1);
        goalList.Add(goal2);
        goalList.Add(goal3);

        agent = GetComponent<NavMeshAgent>();
        goalPoint = goalList[goalListCount];
        direction = (goalPoint.position - this.transform.position).normalized;
        //agent.destination = goalPoint.position;

        _prevRotation = this.transform.rotation;
        
    }

    // Update is called once per frame
    void Update()
    {
        /* 正面・右・上向き */
        Vector3 forward = goalPoint.forward;
        Vector3 right = goalPoint.right;
        Vector3 up = goalPoint.up;

        /* 後ろ・左・下向き */
        Vector3 back = -forward;
        Vector3 left = -right;
        Vector3 down = -up;

        /*
        // 現在フレームの姿勢を取得
        var rotation = transform.rotation;
        // 前フレームからの回転量を求める
        var diffRotation = Quaternion.Inverse(_prevRotation) * rotation;
        // 回転した角度と軸（ローカル空間）を求める
        diffRotation.ToAngleAxis(out var angle, out var axis);
        // 回転角度が0以外なら回転しているとみなす
        IsRotating = !Mathf.Approximately(angle, 0);
        // 回転角度から角速度を計算
        AngularVelocity = angle / Time.deltaTime;
        // ローカル空間の回転軸をワールド空間に変換
        Axis = rotation * axis;
        // 前フレームの姿勢を更新
        _prevRotation = rotation;
        */

        /* 回転操作 */
        if (directionCheckFlag)
        {
            direction = (goalPoint.position - this.transform.position).normalized;
            directionCheckFlag = false;
        }
        //this.transform.LookAt(goalPoint.position);
        
        //Quaternion rot = Quaternion.LookRotation(goalPoint.position, Vector3.up);
        //this.transform.rotation = Quaternion.Slerp(this.transform.rotation, rot, Time.deltaTime * RotateSpeed);

        diffVec  = direction - this.transform.forward;
        if (!navMotionCheckFlag)
        {
            Debug.Log(Mathf.Abs(diffVec.z));
            if (Mathf.Abs(diffVec.z) > 0.001f)
            {
                //Debug.Log("角速度 " + AngularVelocity);
                agent.speed = 0f;
                var angle2 = RotateSpeed * Time.deltaTime;
                this.transform.rotation = Quaternion.AngleAxis(angle2, Vector3.up) * this.transform.rotation;
            }
            else
            {
                navMotionCheckFlag = true;
                Invoke("StartNavMeshMove", 0.2f);
            }
        }
        MovingNavMesh();
    }
    public void StartNavMeshMove()
    {
        agent.destination = goalPoint.position;
        agent.speed = 10f;
    }
    public void MovingNavMesh()
    {
        /* ゴール移動処理 */
        if (navMotionCheckFlag)
        {
            float dis = Vector3.Distance(goalPoint.position, this.transform.position);
            float disX = goalPoint.position.x - this.transform.position.x;
            Debug.Log(Mathf.Abs(disX));
            if (Mathf.Abs(disX) < 1.5f)
            {
                agent.speed = 0f;
                if (goalList.Count > goalListCount)
                {
                    if (goalList[goalListCount] == goalPoint)
                    {
                        Debug.Log("Button Check OK");
                        goalListCount++;
                        goalPoint = goalList[goalListCount];
                        navMotionCheckFlag = false;
                        directionCheckFlag = true;
                    }
                }
            }
        }
    }
}
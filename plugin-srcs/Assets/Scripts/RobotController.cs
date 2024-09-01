using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;



public class RobotController : MonoBehaviour
{
    private int goalListCount = 0;
    private Transform goalPoint;
    public Transform goal1;
    public Transform goal2;
    public Transform goal3;
    public List<Transform> goalList;
    private Vector3 heading;
    private Vector3 diffVec;
    public bool directionCheckFlag = true;
    public bool navMotionCheckFlag = false;
    public float RotateSpeed = 10f;
    public bool canMoveRobot = false;
    // Start is called before the first frame update
    void Start()
    {
        goalList.Add(goal1);
        goalList.Add(goal2);
        goalList.Add(goal3);
        goalPoint = goalList[goalListCount];
        heading = goalPoint.position - this.transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        float angleSpeed = RotateSpeed;
        Vector3 RobotFoward = this.transform.forward;
        //Debug.Log(RobotFoward);
        //Debug.Log(heading.normalized);
        Vector3 diffRotaion = (heading.normalized - RobotFoward)*100;
        if (canMoveRobot)
        {
            MoveRobotFoward();
        }
        else
        {
            //if (diffRotaion > 0.001f)
            if (Mathf.Abs(diffRotaion.x) > 0.1f)
            {
                //Debug.Log(diffRotaion.x);
                //Debug.Log(diffRotaion.z);
                this.transform.Rotate(0, angleSpeed, 0);
            }
            else
            {
                Debug.Log(diffRotaion.x);
                Debug.Log(diffRotaion.z);
                this.transform.Rotate(0, 0, 0);
                canMoveRobot = true;
            }
        }
    }
    public void MoveRobotFoward()
    {
        this.transform.position += this.transform.forward * 0.01f;
        float dis = Vector3.Distance(goalPoint.position, this.transform.position);
        //Debug.Log(Mathf.Abs(dis));
        if (Mathf.Abs(dis) < 1.0f)
        {
            if (goalList.Count > goalListCount)
            {
                goalListCount++;
                goalPoint = goalList[goalListCount];
                heading = goalPoint.position - this.transform.position;
            }
            else
            {
                goalPoint = null;
            }
            canMoveRobot = false;
        }
    }
}

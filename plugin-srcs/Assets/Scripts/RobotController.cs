using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;



public class RobotController : MonoBehaviour
{
    private int goalListCount = 0;
    public Transform obstaclePoint;
    private bool isObstacleState = false;
    private Transform goalPoint;
    private bool isGoalState = false;
    private bool isGoalRestartState = false;
    private int hitCount = 0;
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
        goalPoint = goalList[goalListCount];
        heading = goalPoint.position - this.transform.position;
    }
    private void SetPrimitiveObject()
    {
        //for (int i = 0; i < goalList.Count; i++)
        //{
        //    GameObject objectCenterPoint = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //    objectCenterPoint.transform.position = goalList[i].position;
        //    objectCenterPoint.transform.localScale = goalList[i].localScale;
        //    objectCenterPoint.transform.name = goalList[i].name + "CenterPoint";
        //    objectCenterPoint.transform.parent = goalList[i];
        //    objectCenterPoint.AddComponent<BoxCollider>();
        //}
    }

    // Update is called once per frame
    void Update()
    {

        RobotControllerNoObstacle();

        //if (Physics.Raycast(ray, out RaycastHit hits, Mathf.Infinity))
        //{
//
        //    if (hit.transform.name == goalPoint.transform.name)
        //    {
        //        Vector3 RobotFoward = this.transform.forward;
        //        heading = goalPoint.position - this.transform.position;
        //        Vector3 diffRotaion = heading.normalized - RobotFoward;
        //        Debug.Log(Mathf.Abs(diffRotaion.x));
        //        if (Mathf.Abs(diffRotaion.x) < 0.004f)
        //        {
        //            canMoveRobot = true;
        //        }
        //    }
        //    //if (hitInfo.transform.name == goalPoint.Find(goalPoint.name + "CenterPoint").transform.name)
        //    //{
        //    //    Debug.Log(goalPoint.Find(goalPoint.name + "CenterPoint").transform.name);
        //    //    Debug.Log(hitInfo.transform.name);
        //    //    Debug.Log(goalPoint.transform.Find(goalPoint.name + "CenterPoint").transform.name);
        //    //    canMoveRobot = true;
        //    //}
        //}

        //Vector3 RobotFoward = this.transform.forward;
        //Debug.Log(RobotFoward);
        //Debug.Log(heading.normalized);
        //Vector3 diffRotaion = (heading.normalized - RobotFoward)*100;
        //if (canMoveRobot)
        //{
        //    this.transform.Rotate(0, 0, 0);
        //    MoveRobotFoward();
        //}
        //else
        //{
        //    //if (diffRotaion > 0.001f)
        //    //if (Mathf.Abs(diffRotaion.x) > 0.1f)
        //    if (!canMoveRobot)
        //    {
        //        //Debug.Log(diffRotaion.x);
        //        //Debug.Log(diffRotaion.z);
        //        if (obstaclePoint != null)
        //        {
        //            //this.transform.Rotate(0, angleSpeed, 0);
        //        }
        //    }
        //    else
        //    {
        //        //Debug.Log(diffRotaion.x);
        //        //Debug.Log(diffRotaion.z);
        //        //this.transform.Rotate(0, 0, 0);
        //        canMoveRobot = true;
        //    }
        //}
    }
    public void RobotControllerTest1()
    {
        Ray ray = new Ray(transform.position, this.transform.forward);
        Debug.DrawRay(gameObject.transform.position, this.transform.forward * 30, Color.blue, 0.01f);
        Debug.DrawRay(gameObject.transform.position, this.transform.right * 30, Color.blue, 0.01f);
        Debug.DrawRay(gameObject.transform.position, - this.transform.right * 30, Color.blue, 0.01f);
        
        RaycastHit[]  hits;
        hits = Physics.RaycastAll(ray, Mathf.Infinity);
        
        float angleSpeed = RotateSpeed;
        if (!isGoalState)
        {
            Debug.Log("! isGoalState");
            this.transform.Rotate(0, angleSpeed, 0);
            for (int i = 0; i < hits.Length; i++)
            {
                    if (!isObstacleState)
                    {
                        if (hits[i].transform.name == goalPoint.transform.name)
                        {
                            isGoalState = true;
                            obstaclePoint = hits[0].transform;
                            hitCount = i;
                        }
                    }
                    if (isObstacleState)
                    {
                        if (hits[i].transform.name == goalPoint.transform.name)
                        {
                            Vector3 RobotFoward = this.transform.forward;
                            heading = goalPoint.position - this.transform.position;
                            Vector3 diffRotaion = heading.normalized - RobotFoward;
                            if (Mathf.Abs(diffRotaion.x) < 0.004f)
                            {
                                canMoveRobot = true;
                            }
                        }
                    }
            }
        }
        if (isGoalState)
        {
            Debug.Log("isGoalState");
            RaycastHit hit;
            if (!isGoalRestartState)
            {
                if (Physics.Raycast(ray, out hit, Mathf.Infinity))
                {
                    if (hit.transform.name == obstaclePoint.name)
                    {
                        this.transform.Rotate(0, angleSpeed, 0);
                    }
                    //else
                    //{
                    //    this.transform.Rotate(0, 0, 0);
                    //    isGoalRestartState = true;
                    //}
                }
                else
                {
                    this.transform.Rotate(0, 0, 0);
                    isGoalRestartState = true;
                }
            }
            if (isGoalRestartState)
            {
                this.transform.position += this.transform.forward * 0.01f;
                Ray obstacleRightRay = new Ray(obstaclePoint.position, obstaclePoint.right);
                Ray obstacleLeftRay = new Ray(obstaclePoint.position, -obstaclePoint.right);
                Debug.DrawRay(obstaclePoint.position, obstaclePoint.right * 30, Color.red, 0.01f);
                Debug.DrawRay(obstaclePoint.position, -obstaclePoint.right * 30, Color.red, 0.01f);
                if (Physics.Raycast(obstacleRightRay, out hit, Mathf.Infinity) || Physics.Raycast(obstacleLeftRay, out hit, Mathf.Infinity))
                {
                    if (hit.transform.name == this.transform.name)
                    {
                        isObstacleState = true;
                        isGoalState = false;
                        isGoalRestartState = false;
                        obstaclePoint = null;
                    }
                }
            }
        }
        if (canMoveRobot)
        {
            Debug.Log("canMoveRobot");
            this.transform.Rotate(0, 0, 0);
            MoveRobotFoward();
        }
    }
    public void RobotControllerNoObstacle()
    {
        Ray ray = new Ray(transform.position, this.transform.forward);
        Debug.DrawRay(gameObject.transform.position, this.transform.forward * 30, Color.blue, 0.01f);
        Debug.DrawRay(gameObject.transform.position, this.transform.right * 30, Color.blue, 0.01f);
        Debug.DrawRay(gameObject.transform.position, - this.transform.right * 30, Color.blue, 0.01f);
        if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
        {
            if (hit.transform.name == goalPoint.transform.name)
            {
                Vector3 RobotFoward = this.transform.forward;
                heading = goalPoint.position - this.transform.position;
                Vector3 diffRotaion = heading.normalized - RobotFoward;
                if (Mathf.Abs(diffRotaion.x) < 0.004f)
                {
                    canMoveRobot = true;
                }
            }
        }

        if (canMoveRobot)
        {
            this.transform.Rotate(0, 0, 0);
            MoveRobotFoward();
        }
        else
        {
            //if (diffRotaion > 0.001f)
            //if (Mathf.Abs(diffRotaion.x) > 0.1f)
            if (!canMoveRobot)
            {
                this.transform.Rotate(0, RotateSpeed, 0);
            }
            else
            {
                //Debug.Log(diffRotaion.x);
                //Debug.Log(diffRotaion.z);
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
        if (Mathf.Abs(dis) < 1.6f)
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
            isObstacleState = false;
        }
    }
}

using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class HakoniwaDataController : MonoBehaviour
    {
        public HakoniwaDataManager hakoniwaDataManager = new HakoniwaDataManager();

        //GameObject
        public GameObject robot;
        public GameObject robotCamera;
        public GameObject realRobot;

        // misc
        private int goalListCount = 0;
        private Transform goalPoint;
        public List<Transform> goalList;
        private Vector3 heading;

        
        public float RotateSpeed = 10f;
        public bool canMoveRobot = false;
        private bool rotateIsOn;
        private bool runningIsOn;

        // 自動動作用
        private bool isAutoMoving = false;
        private Coroutine autoMoveCoroutine = null;
        public float autoRotateTimeA = 1.0f; // Aボタン旋回秒数
        public float autoForwardTimeA = 2.0f; // Aボタン前進秒数
        public float autoRotateTimeB = 0.5f; // Bボタン旋回秒数
        public float autoBackwardTimeB = 1.5f; // Bボタン後進秒数


        // Start is called before the first frame update
        void Start()
        {
            //goalPoint = goalList[goalListCount];
            //heading = goalPoint.position - this.transform.position;
            //rotateIsOn = true;
        }
        
        public int count_i = 0;
        // Update is called once per frame
        void Update()
        {
            // ROSトピックによる位置・姿勢同期は常に行う
            robot.transform.localPosition = new Vector3((float)(hakoniwaDataManager.CameraPosition_x), 0.0f, (float)(hakoniwaDataManager.CameraPosition_z));
            robot.transform.localRotation = Quaternion.Euler((float)hakoniwaDataManager.CameraQuaternion_x, (float)hakoniwaDataManager.CameraQuaternion_y + 90.0f, (float)hakoniwaDataManager.CameraQuaternion_z);

            // 自動動作中はROSトピックからの速度指令を無視
            if (isAutoMoving)
            {
                return;
            }

            // ボタンA押下時（押しっぱなしでも一度だけ）
            if (hakoniwaDataManager.buttonA_flag && autoMoveCoroutine == null)
            {
                autoMoveCoroutine = StartCoroutine(AutoMoveSequence(true));
                return;
            }
            // ボタンB押下時（押しっぱなしでも一度だけ）
            if (hakoniwaDataManager.buttonB_flag && autoMoveCoroutine == null)
            {
                autoMoveCoroutine = StartCoroutine(AutoMoveSequence(false));
                return;
            }

            // 通常の手動操作
            if (Input.GetKeyDown(KeyCode.LeftArrow))
            {
                hakoniwaDataManager.angular_z = 0.1f;
            }
            if (Input.GetKeyDown(KeyCode.RightArrow))
            {
                hakoniwaDataManager.angular_z = -0.1f;
            }
            if (Input.GetKeyDown(KeyCode.UpArrow))
            {
                hakoniwaDataManager.linear_x = 0.06f;
            }
            if (Input.GetKeyDown(KeyCode.DownArrow))
            {
                hakoniwaDataManager.linear_x = -0.06f;
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                hakoniwaDataManager.linear_x = 0.00f;
                hakoniwaDataManager.angular_z = 0.0f;
            }

            if (goalPoint != null)
            {
                RobotControllerNoObstacle();
            }
        }

        // 自動動作コルーチン
        private IEnumerator AutoMoveSequence(bool isA)
        {
            isAutoMoving = true;
            // A:前進、B:後進
            float rotateTime = isA ? autoRotateTimeA : autoRotateTimeB;
            float moveTime = isA ? autoForwardTimeA : autoBackwardTimeB;
            float moveSpeed = isA ? 0.06f : -0.06f;

            // 旋回（正方向）
            hakoniwaDataManager.linear_x = 0.0f;
            hakoniwaDataManager.angular_z = 0.5f;
            yield return new WaitForSeconds(rotateTime);

            // 前進または後進
            hakoniwaDataManager.linear_x = moveSpeed;
            hakoniwaDataManager.angular_z = 0.0f;
            yield return new WaitForSeconds(moveTime);

            // 停止
            hakoniwaDataManager.linear_x = 0.0f;
            hakoniwaDataManager.angular_z = 0.0f;

            isAutoMoving = false;
            autoMoveCoroutine = null;
        }
        public void robotRotateAndPostionSync()
        {
            //this.transform.localPosition = realRobot.transform.localPosition;
            this.transform.position = realRobot.transform.position + new Vector3(0, 0, 1);
            this.transform.localRotation = realRobot.transform.localRotation;
            this.transform.Rotate(0, -90, 0);
        }

        public void RobotControllerNoObstacle()
        {
            Ray ray = new Ray(robotCamera.transform.position, robotCamera.transform.forward);
            Debug.DrawRay(robotCamera.gameObject.transform.position, robotCamera.transform.forward * 30, Color.blue, 0.01f);
            Debug.DrawRay(robotCamera.gameObject.transform.position, robotCamera.transform.right * 30, Color.blue, 0.01f);
            Debug.DrawRay(robotCamera.gameObject.transform.position, - robotCamera.transform.right * 30, Color.blue, 0.01f);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
            {
                //Debug.Log(hit.transform.name == goalPoint.transform.name);
                if (hit.transform.name == goalPoint.transform.name)
                {
                    
                    Vector3 RobotFoward = robot.transform.forward;
                    heading = goalPoint.position - robotCamera.transform.position;
                    Vector3 diffRotaion = heading.normalized - RobotFoward;
                    if (Mathf.Abs(diffRotaion.x) < 1.404f)
                    {
                        canMoveRobot = true;
                    }
                }
            }

            if (canMoveRobot)
            {
                //this.transform.Rotate(0, 0, 0);
                hakoniwaDataManager.angular_z = 0.0f;
                hakoniwaDataManager.linear_x = 0.06f;
                //this.target_velocity = 0;
                //this.target_rotation_angle_rate = 0;
                runningIsOn = true;
                MoveRobotFoward();
            }
            else
            {
                //if (diffRotaion > 0.001f)
                //if (Mathf.Abs(diffRotaion.x) > 0.1f)
                if (!canMoveRobot)
                {
                    //this.transform.Rotate(0, 0.1f, 0);
                    if (rotateIsOn)
                    {
                        Debug.Log("rotateIsOn");
                        //this.target_rotation_angle_rate += delta_angle;
                        hakoniwaDataManager.angular_z = 0.1f;
                        rotateIsOn = false;
                    }
                }
                else
                {
                    //Debug.Log(diffRotaion.x);
                    //Debug.Log(diffRotaion.z);
                    //this.transform.Rotate(0, 0, 0);
                    //this.target_rotation_angle_rate = 0;
                    canMoveRobot = true;
                }
            }
        }
        public void MoveRobotFoward()
        {
            //this.transform.position += this.transform.forward * 0.01f;
            //this.transform.position += this.transform.forward * (float)delta_vel;
            if (runningIsOn)
            {
                //this.target_velocity += delta_vel;
                runningIsOn = false;
            }
            float dis = Vector3.Distance(goalPoint.position, robotCamera.transform.position);
            Debug.Log(dis);
            //Debug.Log(Mathf.Abs(dis));
            if (Mathf.Abs(dis) < 0.06f)
            {
                Debug.Log("GOAL OK");
                hakoniwaDataManager.linear_x = 0.0f;
                count_i = 0;
                //if (goalList.Count > goalListCount)
                //{
                //    goalListCount++;
                //    goalPoint = goalList[goalListCount];
                //    heading = goalPoint.position - robotCamera.transform.position;
                //}
                //else
                //{
                //    goalPoint = null;
                //}
                goalPoint = null;
                canMoveRobot = false;
                //rotateIsOn = true;
            }
        }
    }
}
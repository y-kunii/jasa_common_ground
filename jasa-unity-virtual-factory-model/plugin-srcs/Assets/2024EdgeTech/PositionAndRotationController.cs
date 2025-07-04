using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class PositionAndRotationController : MonoBehaviour, IRobotPartsController, IRobotPartsConfig
    {
        private GameObject root;
        private string root_name;
        private IRobotPartsMotor [] motors = new IRobotPartsMotor[(int)MotorType.MotorType_Num];
        private PduIoConnector pdu_io;
        private IPduReader pdu_reader;
        public float steering_sensitivity = 1.5f;                // ?o???l

        public string topic_type = "geometry_msgs/Pose";
        public string topic_name = "pose_data";
        public string roboname = "Camera";
        public int update_cycle = 10;
        public float motor_interval_distance = 0.160f; // 16cm

        private int count = 0;

        public RosTopicMessageConfig[] getRosConfig()
        {
            RosTopicMessageConfig[] cfg = new RosTopicMessageConfig[1];
            cfg[0] = new RosTopicMessageConfig();
            cfg[0].topic_message_name = this.topic_name;
            cfg[0].topic_type_name = this.topic_type;
            cfg[0].sub = true;
            return cfg;
        }
        public void Initialize(System.Object obj)
        {
            GameObject tmp = null;
            try
            {
                tmp = obj as GameObject;
            }
            catch (InvalidCastException e)
            {
                Debug.LogError("Initialize error: " + e.Message);
                return;
            }

            if (this.root == null)
            {
                this.root = tmp;
                this.root_name = string.Copy(this.root.transform.name);
                this.pdu_io = PduIoConnector.Get(root_name);
                if (this.pdu_io == null)
                {
                    throw new ArgumentException("can not found pdu_io:" + roboname);
                }
                var pdu_reader_name = roboname + "_" + this.topic_name + "Pdu";
                this.pdu_reader = this.pdu_io.GetReader(pdu_reader_name);
                if (this.pdu_reader == null)
                {
                    throw new ArgumentException("can not found pdu_reader:" + pdu_reader_name);
                }
                //this.motors[(int)MotorType.MotorType_Left] = this.transform.Find("Interface-L").GetComponentInChildren<IRobotPartsMotor>();
                //this.motors[(int)MotorType.MotorType_Right] = this.transform.Find("Interface-R").GetComponentInChildren<IRobotPartsMotor>();
                //Debug.Log("motor left=" + this.motors[(int)MotorType.MotorType_Left]);
                //Debug.Log("motor right=" + this.motors[(int)MotorType.MotorType_Right]);
            }
            //for (int i = 0; i < this.motors.Length; i++)
            //{
            //    if (this.motors[i] != null)
            //    {
            //        this.motors[i].Initialize(root);
            //    }
            //}
            this.count = 0;
        }

        public GameObject gameObjectForRobot;
        //private void Update()
        //{
        //    //gameObjectForRobot.transform.localPosition = new Vector3(1.0f, 1.0f, 10.0f);
        //}

        public double position_x;
        public double position_y;
        public double position_z;

        public double quaternion_x;
        public double quaternion_y;
        public double quaternion_z;
        public double quaternion_w;
        public void DoControl()
        {
            this.count++;
            if (this.count < this.update_cycle)
            {
                return;
            }
            this.count = 0;
            // pdu position data
            position_x = this.pdu_reader.GetReadOps().Ref("position").GetDataFloat64("x");
            position_y = this.pdu_reader.GetReadOps().Ref("position").GetDataFloat64("y");
            position_z = this.pdu_reader.GetReadOps().Ref("position").GetDataFloat64("z");
            Debug.Log("position_x " + position_x);

            // pdu rotate data
            quaternion_x = this.pdu_reader.GetReadOps().Ref("orientation").GetDataFloat64("x");
            quaternion_y = this.pdu_reader.GetReadOps().Ref("orientation").GetDataFloat64("y");
            quaternion_z = this.pdu_reader.GetReadOps().Ref("orientation").GetDataFloat64("z");
            quaternion_w = this.pdu_reader.GetReadOps().Ref("orientation").GetDataFloat64("w");
        }
        public IoMethod io_method = IoMethod.RPC;
        public CommMethod comm_method = CommMethod.UDP;
        public RoboPartsConfigData[] GetRoboPartsConfig()
        {
            RoboPartsConfigData[] configs = new RoboPartsConfigData[1];
            configs[0] = new RoboPartsConfigData();
            configs[0].io_dir = IoDir.READ;
            configs[0].io_method = this.io_method;
            configs[0].value.org_name = this.topic_name;
            configs[0].value.type = this.topic_type;
            configs[0].value.class_name = ConstantValues.pdu_reader_class;
            configs[0].value.conv_class_name = ConstantValues.conv_pdu_reader_class;
            configs[0].value.pdu_size = ConstantValues.Pose_pdu_size;
            configs[0].value.write_cycle = this.update_cycle;
            configs[0].value.method_type = this.comm_method.ToString();
            return configs;
        }
    }
}


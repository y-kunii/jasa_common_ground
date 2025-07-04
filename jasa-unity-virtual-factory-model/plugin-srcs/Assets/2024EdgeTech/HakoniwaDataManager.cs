using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Hakoniwa.GUI;

namespace Hakoniwa.PluggableAsset.Assets.Robot.Parts
{
    public class HakoniwaDataManager : MonoBehaviour
    {
        // pdu position data
        public double CameraPosition_x;
        public double CameraPosition_y;
        public double CameraPosition_z;

        // pdu rotate data
        public double CameraQuaternion_x;
        public double CameraQuaternion_y;
        public double CameraQuaternion_z;
        public double CameraQuaternion_w;

        // pdu robot data
        public double linear_x;
        public double angular_z;

        // pdu button data
        public bool buttonA_flag;
        public bool buttonB_flag;

        // misc
        public PositionAndRotationController positionAndRotationController = new PositionAndRotationController();
        public RobotController robotController = new RobotController();
        
        public ButtonNotificationController buttonNotificationControllerA = new ButtonNotificationController();
        public ButtonNotificationController buttonNotificationControllerB = new ButtonNotificationController();

        void Start()
        {

        }

        void Update()
        {
            CameraPosition_x = positionAndRotationController.position_x;
            CameraPosition_y = positionAndRotationController.position_y;
            CameraPosition_z = positionAndRotationController.position_z;

            CameraQuaternion_x = positionAndRotationController.quaternion_x;
            CameraQuaternion_y = positionAndRotationController.quaternion_y;
            CameraQuaternion_z = positionAndRotationController.quaternion_z;
            CameraQuaternion_w = positionAndRotationController.quaternion_w;

            //linear_x = robotController.linear_x;
            //angular_z = robotController.angular_z;

            robotController.linear_x = linear_x;
            robotController.angular_z = angular_z;

            buttonA_flag = buttonNotificationControllerA.button_flag;
            Debug.Log("buttonA_flag " + buttonA_flag);
            buttonB_flag = buttonNotificationControllerB.button_flag;
            Debug.Log("buttonB_flag " + buttonB_flag);

        }

    }
}

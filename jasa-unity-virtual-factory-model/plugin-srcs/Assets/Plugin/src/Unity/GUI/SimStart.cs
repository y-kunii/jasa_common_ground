﻿using Hakoniwa.Core;
using Hakoniwa.Core.Simulation;
using Hakoniwa.PluggableAsset;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using Debug = UnityEngine.Debug;

namespace Hakoniwa.GUI
{
    public class SimStart : MonoBehaviour
    {
        Text my_text;
        Button my_btn;
        bool isResetHappened = false;
        private enum SimCommandStatus
        {
            WaitStart = 0,
            WaitStop = 1,
            WaitReset = 2,
        }
        private SimCommandStatus cmd_status = SimCommandStatus.WaitStart;

        void Start()
        {
            var obj = GameObject.Find("StartButton");
            my_btn = obj.GetComponentInChildren<Button>();
            my_text = obj.GetComponentInChildren<Text>();
            my_btn.interactable = false;
        }
        public void ResetEvent()
        {
            isResetHappened = true;
        }
        void Update()
        {
            ISimulationController simulator = WorldController.Get();
            var state = simulator.GetState();
            if (AssetConfigLoader.core_config.cpp_mode != null)
            {
                my_btn.interactable = true;
            }
            else
            {
                //button enabler
                int count = simulator.RefOutsideAssetListCount();
                //int count = 0;
                if (count == 0)
                {
                    if (AssetConfigLoader.core_config.outside_assets == null)
                    {
                        my_btn.interactable = true;
                    }
                    else
                    {
                        my_btn.interactable = false;
                    }
                }
                else if ((state != SimulationState.Running) && (state != SimulationState.Stopped))
                {
                    my_btn.interactable = false;
                    return;
                }
                else
                {
                    my_btn.interactable = true;
                }

            }
            //cmd status changer
            switch (cmd_status)
            {
                case SimCommandStatus.WaitStart:
                    if (state == SimulationState.Running)
                    {
                        my_text.text = "STOP";
                        cmd_status = SimCommandStatus.WaitStop;
                    }
                    isResetHappened = false;
                    break;
                case SimCommandStatus.WaitStop:
                    if (state == SimulationState.Stopped)
                    {
                        my_text.text = "RESET";
                        cmd_status = SimCommandStatus.WaitReset;
                    }
                    break;
                case SimCommandStatus.WaitReset:
                    if (isResetHappened)
                    {
                        my_text.text = "START";
                        cmd_status = SimCommandStatus.WaitStart;
                    }
                    break;
                default:
                    break;
            }
        }
        public void OnButtonClick()
        {
            ISimulationController simulator = WorldController.Get();
            switch (cmd_status)
            {
                case SimCommandStatus.WaitStop:
                    simulator.Stop();
                    my_btn.interactable = false; 
                    break;
                case SimCommandStatus.WaitReset:
                    simulator.Reset();
                    my_btn.interactable = false;
                    break;
                case SimCommandStatus.WaitStart:
                    simulator.Start();
                    my_btn.interactable = false;
                    break;
                default:
                    break;
            }
        }
    }
}

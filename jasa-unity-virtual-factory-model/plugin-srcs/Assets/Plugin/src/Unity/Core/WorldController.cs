﻿#define NO_USE_GRPC
#if NO_USE_GRPC
#else
using Hakoniwa.Core.Rpc;
#endif
using Hakoniwa.Core.Simulation;
using Hakoniwa.Core.Simulation.Environment;
using Hakoniwa.Core.Utils;
using Hakoniwa.Core.Utils.Logger;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System.Text;
using System.Runtime.InteropServices;

namespace Hakoniwa.Core
{
    class UnitySimulator : IInsideWorldSimulatior
    {
        private long delta_time;
        public UnitySimulator()
        {
            delta_time = (long)Math.Round((double)Time.fixedDeltaTime * 1000000.0f);
        }
        public void DoSimulation()
        {
            Physics.Simulate(Time.fixedDeltaTime);
        }
        public long GetDeltaTimeUsec()
        {
            return delta_time;
        }
    }

    public class WorldController : MonoBehaviour
    {
        private GameObject root;
        public long maxDelayTime = 20000; /* usec */
        private static ISimulationController isim;
        private static ISimulationAssetManager iasset;

        public static ISimulationController Get()
        {
            return isim;
        }
        private void InitHakoniwa()
        {
            this.root = GameObject.Find("Robot");
#if UNITY_EDITOR
            string filePath = Directory.GetCurrentDirectory();
#else
            string filePath = AppDomain.CurrentDomain.BaseDirectory;
#endif
            //Debug.Log(filePath);
            //string configPath = filePath + System.IO.Path.DirectorySeparatorChar + "core_config.json";
            string configPath = "./core_config.json";
            AssetConfigLoader.Load(configPath);
            if (AssetConfigLoader.core_config.cpp_mode != null) {
                Debug.Log("cpp_mode:" + AssetConfigLoader.core_config.cpp_mode);
                Debug.Log("cpp_asset_name:" + AssetConfigLoader.core_config.cpp_asset_name);
                isim = SimulationControllerFactory.Get(AssetConfigLoader.core_config.cpp_asset_name);
            }
            else {
#if NO_USE_GRPC
                throw new NotSupportedException("ERROR: asset_rpc is not supported..");
#else
                Debug.Log("cpp_mode: None");
                isim = SimulationControllerFactory.Get(null);
                RpcServer.StartServer(AssetConfigLoader.core_config.core_ipaddr, AssetConfigLoader.core_config.core_portno);
                SimulationController.Get().SetSimulationWorldTime(
                    this.maxDelayTime,
                    (long)(Time.fixedDeltaTime * 1000000f));
#endif
            }
            Debug.Log("HakoniwaCore START");

            iasset = isim.GetAssetManager();
            isim.RegisterEnvironmentOperation(new UnityEnvironmentOperation());

            isim.SaveEnvironment();

            Debug.Log("childcount=" + this.transform.childCount);
            for (int i = 0; i < this.transform.childCount; i++)
            {
                Transform child = this.transform.GetChild(i);
                Debug.Log(child.name);
                var ctrl = child.GetComponentInChildren<IInsideAssetController>();
                ctrl.Initialize();
                AssetConfigLoader.AddInsideAsset(ctrl);
                iasset.RegisterInsideAsset(child.name);
            }
            isim.SetInsideWorldSimulator(new UnitySimulator());
            Physics.autoSimulation = false;

        }
        public void Login(LoginRobotInfoType robo)
        {
            string path = "Hakoniwa/Robots/" + robo.robotype;
            var p = Resources.Load<GameObject>(path);
            if (p == null)
            {
                throw new InvalidDataException("ERROR: path is not found:" + path);
            }
            IInsideAssetController ctrl = null;
            Vector3 pos = new Vector3(robo.pos.X, robo.pos.Y, robo.pos.Z);
            var articulationbodies = p.GetComponentsInChildren<ArticulationBody>();
            if ((articulationbodies != null) && articulationbodies.Length > 0)
            {
                var dir = Quaternion.Euler(robo.angle.X, robo.angle.Y, robo.angle.Z) * Vector3.forward;
                Quaternion qangle = Quaternion.LookRotation(dir);
                foreach (var articulationBody in articulationbodies)
                {
                    if (articulationBody.isRoot)
                    {
                        var instance = Instantiate(p, pos, qangle) as GameObject;
                        //var instance = Instantiate(p, pos, Quaternion.identity) as GameObject;
                        articulationBody.velocity = Vector3.zero;
                        articulationBody.angularVelocity = Vector3.zero;
                        articulationBody.TeleportRoot(pos, qangle);
                        instance.name = robo.roboname;
                        instance.transform.parent = this.root.transform;

                        ctrl = instance.GetComponentInChildren<IInsideAssetController>();
                        break;
                    }
                }
            }
            else
            {
                var instance = Instantiate(p, pos, Quaternion.identity) as GameObject;
                //RigidBody
                instance.transform.Rotate(new Vector3(robo.angle.X, robo.angle.Y, robo.angle.Z));
                instance.name = robo.roboname;
                instance.transform.parent = this.root.transform;

                ctrl = instance.GetComponentInChildren<IInsideAssetController>();
            }
            ctrl.Initialize();
            AssetConfigLoader.AddInsideAsset(ctrl);
            iasset.RegisterInsideAsset(robo.roboname);
        }
        void Start()
        {
            try
            {
                this.InitHakoniwa();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        void FixedUpdate()
        {
            try
            {
                isim.Execute();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        void OnApplicationQuit()
        {
            if (AssetConfigLoader.core_config.cpp_mode != null) {
                Debug.Log("cpp_mode:" + AssetConfigLoader.core_config.cpp_mode);
                Debug.Log("OnApplicationQuit:enter");
                if (AssetConfigLoader.core_config.cpp_mode.Equals("asset_rpc"))
                {
                    //nothing to do.
                }
                else
                {
                    HakoCppWrapper.asset_unregister(AssetConfigLoader.core_config.cpp_asset_name);
                }
            }
        }
    }
}

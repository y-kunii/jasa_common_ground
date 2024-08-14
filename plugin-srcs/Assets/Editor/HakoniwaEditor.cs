using UnityEngine;
using UnityEditor;
using System.IO;
using Newtonsoft.Json;
using Hakoniwa.PluggableAsset.Assets.Environment;
using Hakoniwa.PluggableAsset;
using System;
using Newtonsoft.Json.Linq;
using Hakoniwa.PluggableAsset.Assets.Robot.Parts;
using System.Text.RegularExpressions;

[System.Serializable]
class HakoniwaPathSettings
{
    public string hakoniwa_base;
    public string settings;
    public string pdu_path;
    public string offset_path;
}

public class HakoniwaEditor : EditorWindow
{
    private static int asset_num;
    private static RosTopicMessageConfigContainer ros_topic_container;
    private static GameObject[] hako_asset_roots;
    private static GameObject[] hako_assets;
    private static LoginRobot login_robots;
    private static JArray micon_settings_json_array = new JArray();
    private static JObject micon_settings_json = new JObject();
    private static HakoniwaPathSettings path;

    static void LoadPath()
    {
#if UNITY_IOS
        string jsonString = File.ReadAllText("./Assets/Resources/hakoniwa_path.json");
#else
        string jsonString = File.ReadAllText("./hakoniwa_path.json");
#endif
        path = JsonConvert.DeserializeObject<HakoniwaPathSettings>(jsonString);
        Debug.Log("hakoniwa_base path=" + path.hakoniwa_base);
        Debug.Log("settings path=" + path.settings);
    }

    static void Init()
    {
        asset_num = 0;
        hako_asset_roots = null;
        hako_assets = null;
        login_robots = null;
        ros_topic_container = new RosTopicMessageConfigContainer();
        ros_topic_container.robot_num = 0;
        ros_topic_container.hakoenv_num = 0;
        micon_settings_json_array = new JArray();
        micon_settings_json = new JObject();

    }

    static string ConvertToJson(RosTopicMessageConfigContainer cfg)
    {
        return JsonConvert.SerializeObject(cfg, Formatting.Indented);
    }
    static private int GetHakoAssetRoots()
    {
        hako_asset_roots = GameObject.FindGameObjectsWithTag("HakoAssetRoot");
        int ret = 0;
        foreach (var root in hako_asset_roots)
        {
            Debug.Log("asset_root:" + root.gameObject.name);
            var hako_env_root = root.GetComponent<IHakoEnv>();
            Debug.Log("hako_env_root: " + hako_env_root);
            if ((hako_env_root != null) && hako_env_root.getRosConfig() != null)
            {
                ret += hako_env_root.getRosConfig().Length;
            }
            var hako_robo_root = root.GetComponent<IRobotParts>();
            Debug.Log("hako_robo_root: " + hako_robo_root);
            if ((hako_robo_root != null) && hako_robo_root.getRosConfig() != null)
            {
                ret += hako_robo_root.getRosConfig().Length;
            }
        }
        return ret;
    }
    static private void GetHakoAssets(int root_num)
    {
        try {
            hako_assets = GameObject.FindGameObjectsWithTag("HakoAsset");
        }
        catch (Exception) {
            Debug.Log("ERROR: Not found TAG: HakoAsset");
            hako_assets = new  GameObject[0];
        }
        int len = 0;
        foreach(var e in hako_assets)
        {
            if (e.GetComponent<IHakoEnv>() != null)
            {
                Debug.Log("HakoEnv: " + e.name);
                len += e.GetComponent<IHakoEnv>().getRosConfig().Length;
            }
        }

        foreach (var root in hako_asset_roots)
        {
            var asset_parts = root.GetComponentsInChildren<IRobotParts>();
            if (asset_parts == null)
            {
                continue;
            }
            var hako_robo_root = root.GetComponent<IRobotParts>();
            foreach (var asset in asset_parts)
            {
                if (asset == hako_robo_root)
                {
                    continue;
                }
                if (asset.getRosConfig() != null)
                {
                    Debug.Log("IRobotParts: " + asset);
                    len += asset.getRosConfig().Length;
                }
            }
        }
        Debug.Log("field_num=" + (len + root_num));
        ros_topic_container.fields = new RosTopicMessageConfig[len + root_num];
    }

    static private void GetHakoAssetConfigs(GameObject root)
    {
        IHakoEnv[] hako_assets = root.GetComponentsInChildren<IHakoEnv>();
        foreach(var asset in hako_assets)
        {
            foreach (var e in asset.getRosConfig())
            {
                e.topic_message_name = root.name + "_" + asset.GetAssetName();
                e.robot_name = root.name;
                ros_topic_container.fields[asset_num] = e;
                asset_num++;
                ros_topic_container.hakoenv_num++;            }
        }
    }
    static private void GetRobotAssetConfig(GameObject root)
    {
        IRobotParts[] robot_assets = root.GetComponentsInChildren<IRobotParts>();
        if (robot_assets == null)
        {
            Debug.Log("No robots");
            return;
        }
        ros_topic_container.robot_num++;

        UnityEngine.Object prefab = PrefabUtility.GetCorrespondingObjectFromSource(root);
        var robo = new LoginRobotInfoType();
        robo.roboname = root.transform.name;
        robo.robotype = prefab.name;
        robo.pos.X = root.transform.position.x;
        robo.pos.Y = root.transform.position.y;
        robo.pos.Z = root.transform.position.z;
        robo.angle.X = root.transform.localEulerAngles.x;
        robo.angle.Y = root.transform.localEulerAngles.y;
        robo.angle.Z = root.transform.localEulerAngles.z;
        int index = login_robots.robos.Length;
        Array.Resize(ref login_robots.robos, index + 1);
        login_robots.robos[index] = robo;
        Debug.Log("roboname=" + root.transform.name);
        Debug.Log("robotype=" + prefab.name);
        Debug.Log("pos=" + root.transform.position);
        foreach (var asset in robot_assets)
        {
            Debug.Log("robot:" + asset.ToString());
            var configs = asset.getRosConfig();
            if (configs == null)
            {
                continue;
            }
            foreach ( var e in configs)
            {
                e.topic_message_name = root.name + "_" + e.topic_message_name;
                //e.topic_message_name = e.topic_message_name;
                e.robot_name = root.name;
                ros_topic_container.fields[asset_num] = e;
                asset_num++;
            }
        }
        IMiconSettings micon_settings = root.GetComponentInChildren<IMiconSettings>();
        if ((micon_settings != null) && (micon_settings.isEnabled()))
        {
            var str = micon_settings.GetSettings(robo.roboname);
            var json_data = JObject.Parse(str);
            micon_settings_json_array.Add(new JObject(json_data));
        }
    }
    static string GetCmdResult(string command)
    {
        System.Diagnostics.Process process = new System.Diagnostics.Process();
        process.StartInfo.FileName = "PowerShell.exe";
        process.StartInfo.Arguments = "wsl -e \"" + command + "\"";
        process.StartInfo.UseShellExecute = false;
        process.StartInfo.RedirectStandardOutput = true;
        process.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
        process.Start();
        return process.StandardOutput.ReadToEnd();
    }

    static void GenerateConfigs(HakoRobotConfigContainer robo_config, bool isRPC, bool isWindows)
    {
        var core_config = new CoreConfig();
        core_config.asset_timeout = 3;
        if (isRPC)
        {
            if (isDebug == false)
            {
                core_config.cpp_mode = "asset_rpc";
                core_config.cpp_asset_name = "UnityAsset";
                core_config.core_portno = 50051;
                core_config.pdu_udp_portno_asset = 54003;
                core_config.pdu_bin_offset_package_dir = path.offset_path;
                if (isWindows)
                {
                    string output = GetCmdResult("cat /etc/resolv.conf");
                    char[] delimiters = new char[] { '\t', ' ', '\n' };
                    string[] array = output.Split(delimiters);
                    core_config.asset_ipaddr = array[array.Length - 2];

                    //eth0
                    output = GetCmdResult("/usr/sbin/ifconfig eth0");
                    Regex regex = new Regex(@"inet\s+(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})");
                    Match match = regex.Match(output);
                    if (match.Success)
                    {
                        core_config.core_ipaddr = match.Groups[1].Value;
                    }
                    else
                    {
                        core_config.core_ipaddr = output;
                        //throw new ArgumentException("Can not found ipaddress from wsl2 eth0");
                    }
                }
            }
            else
            {
                core_config.cpp_mode = "asset_rpc_cpp";
                //TODO
            }
        }
        else // SHM
        {
            core_config.cpp_mode = "asset_rpc_cpp";
            core_config.cpp_asset_name = "UnityAsset";
            core_config.pdu_bin_offset_package_dir = path.offset_path;
        }

        //inside_asset
        HakoConfigCreator.CreateInsideAsset(robo_config, core_config);

        //Pdu reader/writer
        HakoConfigCreator.CreatePduReaderWriter(robo_config, core_config);


        if (isDebug == false)
        {
            //rpc_method, shm_method
            HakoConfigCreator.CreateRpcMethod(robo_config, core_config);
            HakoConfigCreator.CreateShmMethod(robo_config, core_config);
        }
        //connector rw
        HakoConfigCreator.CreateConnector(robo_config, core_config, isDebug);


        //pdu_configs
        HakoConfigCreator.CreatePduConfig(ros_topic_container, core_config, path.pdu_path);
        core_config.ros_topics_path = "./RosTopics.json";

        //core_config
        HakoConfigCreator.CreateCoreConfig(core_config);
    }
    static bool isDebug = false;
    [MenuItem("Window/Hakoniwa/Generate")]
    static void GenerateAssetConfig()
    {
        isDebug = false;
        AssetsUpdateCommon();
    }
    [MenuItem("Window/Hakoniwa/GenerateDebug")]
    static void GenerateDebugAssetConfig()
    {
        isDebug = true;
        AssetsUpdateCommon();
    }
    static void AssetsUpdateCommon()
    {
        Init();
        LoadPath();
        int root_num = GetHakoAssetRoots();
        Debug.Log("assets root_num:" + root_num);
        GetHakoAssets(root_num);
        asset_num = 0;

        login_robots = new LoginRobot();
        login_robots.robos = new LoginRobotInfoType[0];
        foreach(var root in hako_asset_roots)
        {
            GetRobotAssetConfig(root);
            GetHakoAssetConfigs(root);
        }
        ros_topic_container.ros_robot_num = ros_topic_container.robot_num - micon_settings_json_array.Count;
        Debug.Log("json:" + ConvertToJson(ros_topic_container));
        AssetConfigLoader.SaveJsonFile<LoginRobot>(path.settings + "/LoginRobot.json", login_robots);
        File.WriteAllText(path.settings + "/RosTopics.json", ConvertToJson(ros_topic_container));

        HakoRobotConfigContainer robo_config = new HakoRobotConfigContainer();
        if (micon_settings_json_array.Count > 0)
        {
            micon_settings_json.Add(new JProperty("robots", micon_settings_json_array));
            File.WriteAllText(path.settings + "/custom.json", micon_settings_json.ToString());
            if (path.hakoniwa_base != null)
            {
                File.WriteAllText(path.hakoniwa_base + "/custom.json", micon_settings_json.ToString());
            }
            robo_config = HakoConfigCreator.GetHakoRobotConfig(micon_settings_json.ToString());
        }
        else
        {
            File.Delete(path.settings + "/custom.json");
        }
#if NO_USE_GRPC
        GenerateConfigs(robo_config, false, false);
#else
        GenerateConfigs(robo_config, true, true);
#endif
    }


    void OnGUI()
    {
        try
        {

        }
        catch (System.FormatException) { }
    }
}

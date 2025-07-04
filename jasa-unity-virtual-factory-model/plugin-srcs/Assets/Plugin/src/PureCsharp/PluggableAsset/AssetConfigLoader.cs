using Hakoniwa.Core.Utils.Logger;
using Hakoniwa.PluggableAsset.Assets;
using Hakoniwa.PluggableAsset.Communication.Channel;
using Hakoniwa.PluggableAsset.Communication.Connector;
using Hakoniwa.PluggableAsset.Communication.Method;
using Hakoniwa.PluggableAsset.Communication.Method.Shm;
using Hakoniwa.PluggableAsset.Communication.Pdu;
using Hakoniwa.PluggableAsset.Communication.Pdu.Raw;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
#if NO_USE_GRPC
#else
using Hakoniwa.PluggableAsset.Communication.Method.Rpc;
using Hakoniwa.PluggableAsset.Communication.Method.Mqtt;
using Hakoniwa.Core.Rpc;
#endif

namespace Hakoniwa.PluggableAsset
{
    public class AssetConfigLoader
    {
        public static CoreConfig core_config;
        private static List<IInsideAssetController> inside_assets = new List<IInsideAssetController>();
        private static List<IOutsideAssetController> outside_assets = new List<IOutsideAssetController>();
        private static List<IPduReader> pdu_readers = new List<IPduReader>();
        private static List<IPduWriter> pdu_writers = new List<IPduWriter>();
        private static List<IIoReader> io_readers = new List<IIoReader>();
        private static List<IIoWriter> io_writers = new List<IIoWriter>();
        private static List<WriterConnector> writer_connectors = new List<WriterConnector>();
        private static List<ReaderConnector> reader_connectors = new List<ReaderConnector>();
        private static List<PduChannelConnector> pdu_channel_connectors = new List<PduChannelConnector>();
        private static List<PduIoConnector> pdu_io_connectors = new List<PduIoConnector>();
        private static List<PduDataConfig> pdu_configs = new List<PduDataConfig>();

        public static ParamScale GetScale()
        {
            if (core_config.param_world_config != null)
            {
                if (core_config.param_world_config.scale != null)
                {
                    return core_config.param_world_config.scale;
                }
            }
            var tmp = new ParamScale();
            tmp.cmdvel = 1.0f;
            tmp.odom = 1.0f;
            tmp.scan = 1.0f;
            return tmp;
        }

        public static RosTopicMessageConfig GetRosTopic(string config_name)
        {
            if (core_config.ros_topics == null)
            {
                throw new ArgumentException("Can not found ros_topics");
            }
            foreach(var e in core_config.ros_topics)
            {
                if (e.topic_message_name.Equals(config_name))
                {
                    return e;
                }
            }
            return null;
        }

        private static PduChannelConnector GetPduChannelConnector(string name)
        {
            foreach (var e in pdu_channel_connectors)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        public static List<PduChannelConnector> RefPduChannelConnector()
        {
            return pdu_channel_connectors;
        }

        private static PduIoConnector GetPduIoConnector(string name)
        {
            foreach (var e in pdu_io_connectors)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        private static IIoReader GetIoReader(string name)
        {
            if (io_readers == null)
            {
                SimpleLogger.Get().Log(Level.ERROR, "GetIoReader(): Can not found io_readers");
                return null;
            }
            foreach (var e in io_readers)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            SimpleLogger.Get().Log(Level.ERROR, "GetIoReader(): Can not found io_readers name=" + name);
            return null;
        }
        private static IIoWriter GetIoWriter(string name)
        {
            if (io_writers == null)
            {
                return null;
            }
            foreach (var e in io_writers)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        private static IPduReader GetIpduReader(string name)
        {
            foreach (var e in pdu_readers)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        private static IPduWriter GetIpduWriter(string name)
        {
            foreach (var e in pdu_writers)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }

        public static PduDataConfig GetPduConfig(string arg_pdu_type_name)
        {
            foreach (var e in AssetConfigLoader.pdu_configs)
            {
                if (e.pdu_type_name.Equals(arg_pdu_type_name))
                {
                    return e;
                }
            }
            if (arg_pdu_type_name.Contains('/'))
            {
                PduDataConfig tmp = new PduDataConfig();
                tmp.pdu_type_name = arg_pdu_type_name;
                tmp.pdu_data_field_path = AssetConfigLoader.core_config.pdu_configs_parent_path + arg_pdu_type_name + ".json";
                PduDataFieldsConfig cfg = null;
#if UNITY_IOS
                string fileNameWithoutExtension = tmp.pdu_data_field_path.Replace("./", "").Replace(".json", "");
                TextAsset jsonTextAsset = Resources.Load<TextAsset>(fileNameWithoutExtension);
                if (jsonTextAsset == null)
                {
                    throw new ArgumentException("can not find path=" + fileNameWithoutExtension);
                }
                string jsonString = jsonTextAsset.text;
#else
                string jsonString = File.ReadAllText(tmp.pdu_data_field_path);
#endif
                cfg = JsonConvert.DeserializeObject<PduDataFieldsConfig>(jsonString);
                tmp.fields = cfg.fields;
                AssetConfigLoader.pdu_configs.Add(tmp);

                return tmp;
            }
            return null;
        }

        private static WriterConnector GetWriterConnector(string name)
        {
            foreach (var e in writer_connectors)
            {
                if (e.Name.Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        private static ReaderConnector GetReaderConnector(string name)
        {
            foreach (var e in reader_connectors)
            {
                if (e.Name.Equals(name))
                {
                    return e;
                }
            }
            return null;
        }

        public static void AddInsideAsset(IInsideAssetController asset)
        {
            inside_assets.Add(asset);
        }
        public static List<IInsideAssetController> GetInsideAssets()
        {
            return inside_assets;
        }
        public static void AddOutsideAsset(IOutsideAssetController asset)
        {
            outside_assets.Add(asset);
        }
        public static IInsideAssetController GetInsideAsset(string name)
        {
            foreach (var e in inside_assets)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }
        public static IOutsideAssetController GetOutsideAsset(string name)
        {
            foreach (var e in outside_assets)
            {
                if (e.GetName().Equals(name))
                {
                    return e;
                }
            }
            return null;
        }

        private static Type GetType(string path, string class_name)
        {
            Type typeinfo = null;
            SimpleLogger.Get().Log(Level.INFO, "path=" + path + " class_name=" + class_name);
            if (path == null)
            {
                try
                {
                    typeinfo = Type.GetType(class_name);
                    SimpleLogger.Get().Log(Level.INFO, "typeinfo=" + typeinfo);
                    if (typeinfo == null)
                    {
                        //see:https://freelyapps.net/unityengine-types-can-no-longer-be-used/
                        try
                        {
                            var loadobj = System.Reflection.Assembly.Load("Assembly-CSharp");
                            SimpleLogger.Get().Log(Level.INFO, "Found Assembly-CSharp");
                            typeinfo = loadobj.GetType(class_name);
                            if (typeinfo == null)
                            {
                                throw new DllNotFoundException();
                            }
                        }
                        catch (Exception)
                        {
                            SimpleLogger.Get().Log(Level.INFO, "Not found Assembly-CSharp");
                            var loadobj = System.Reflection.Assembly.Load("Hakoniwa.PluggableAsset.Communication");
                            if (loadobj != null)
                            {
                                typeinfo = loadobj.GetType(class_name);
                                SimpleLogger.Get().Log(Level.INFO, "Found Hakoniwa.PluggableAsset.Communication");
                            }
                            else
                            {
                                SimpleLogger.Get().Log(Level.INFO, "Not found Hakoniwa.PluggableAsset.Communication");
                            }
                        }
                    }
                    if (typeinfo == null)
                    {
                        throw new InvalidDataException("ERROR: can not found class=" + class_name);
                    }
                    SimpleLogger.Get().Log(Level.INFO, "load typeinfo" + typeinfo);
                }
                catch (Exception)
                {
                    throw new InvalidDataException("ERROR: can not found class_name=" + class_name);
                }
            }
            else
            {
                var asm = Assembly.LoadFrom(path);
                typeinfo = asm.GetType(class_name);
                if (typeinfo == null)
                {
                    throw new InvalidDataException("ERROR: can not found class=" + class_name);
                }
            }
            return typeinfo;
        }

        private static object ClassLoader(string path, string class_name, string arg1)
        {
            Type typeinfo = AssetConfigLoader.GetType(path, class_name);
            if (arg1 == null)
            {
                SimpleLogger.Get().Log(Level.INFO, "activate class_name=" + class_name);
                return Activator.CreateInstance(typeinfo);
            }
            else
            {
                return Activator.CreateInstance(typeinfo, arg1);

            }
        }
        private static void LoadPduConfig(PduDataConfig config)
        {
            PduDataFieldsConfig cfg = null;
            if (config.pdu_data_field_path != null)
            {
#if UNITY_IOS
                string fileNameWithoutExtension = config.pdu_data_field_path.Replace("./", "").Replace(".json", "");
                TextAsset jsonTextAsset = Resources.Load<TextAsset>(fileNameWithoutExtension);
                if (jsonTextAsset == null)
                {
                    throw new ArgumentException("can not find path=" + fileNameWithoutExtension);
                }
                string jsonString = jsonTextAsset.text;
#else
                string jsonString = File.ReadAllText(config.pdu_data_field_path);
#endif
                cfg = JsonConvert.DeserializeObject<PduDataFieldsConfig>(jsonString);
                config.fields = cfg.fields;
            }
            SimpleLogger.Get().Log(Level.DEBUG, "LoadPduConfig(): pdu_type_name=" + config.pdu_type_name);
            AssetConfigLoader.pdu_configs.Add(config);
        }

        public static T LoadJsonFile<T>(string filepath)
        {
            try
            {
#if UNITY_IOS
                string fileNameWithoutExtension = filepath.Replace("./", "").Replace(".json", "");
                TextAsset jsonTextAsset = Resources.Load<TextAsset>(fileNameWithoutExtension);
                if (jsonTextAsset == null)
                {
                    throw new ArgumentException("can not find path=" + fileNameWithoutExtension);
                }
                string jsonString = jsonTextAsset.text;
#else
                string jsonString = File.ReadAllText(filepath);
#endif
                Debug.Log("filepath=" + filepath);
                var cfg = JsonConvert.DeserializeObject<T>(jsonString);
                SimpleLogger.Get().Log(Level.INFO, "jsonstring=" + jsonString);
                return cfg;
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        public static void SaveJsonFile<T>(string filepath, T json_data)
        {
            try
            {
                string json = JsonConvert.SerializeObject(json_data, Formatting.Indented);
                File.WriteAllText(filepath, json);
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        private static void LoadPduConfig(string filepath)
        {
            if (filepath != null)
            {
                core_config.pdu_configs = LoadJsonFile<PduDataConfig[]>(filepath);
            }
            if (core_config.pdu_configs != null)
            {
                foreach (var cfg in core_config.pdu_configs)
                {
                    AssetConfigLoader.LoadPduConfig(cfg);
                }
            }
        }
        private static void LoadPduWriters(string filepath)
        {
            if (filepath != null)
            {
                core_config.pdu_writers = LoadJsonFile<PduWriterConfig[]>(filepath);
            }
            SimpleLogger.Get().Log(Level.DEBUG, "LoadPduWriters(): filepath=" + filepath);
            SimpleLogger.Get().Log(Level.DEBUG, "LoadPduWriters(): pdu_writers=" + core_config.pdu_writers);
            foreach (var pdu in core_config.pdu_writers)
            {
                SimpleLogger.Get().Log(Level.DEBUG, "LoadPduWriters(): topic_message_name=" + pdu.pdu_config_name);
                IPduWriter ipdu = null;
                if (pdu.topic_message_name == null)
                {
                    Pdu pdu_data = new Pdu(pdu.pdu_config_name);
                    if (pdu_data == null)
                    {
                        throw new InvalidDataException("ERROR: can not found pdu=" + pdu.name);
                    }
                    Type typeinfo = AssetConfigLoader.GetType(pdu.path, pdu.class_name);
                    ipdu = Activator.CreateInstance(typeinfo, pdu_data, pdu.name) as IPduWriter;
                    SimpleLogger.Get().Log(Level.INFO, "pdu writer loaded:" + pdu.class_name);
                    if (ipdu == null)
                    {
                        throw new InvalidDataException("ERROR: can not found classname=" + pdu.class_name);
                    }
                    if (pdu.conv_class_name != null)
                    {
                        IPduWriterConverter conv = AssetConfigLoader.ClassLoader(pdu.conv_path, pdu.conv_class_name, null) as IPduWriterConverter;
                        ipdu.SetConverter(conv);
                    }
                }
                else
                {
                    Pdu topic = new Pdu(pdu.pdu_config_name);
                    if (topic == null)
                    {
                        throw new InvalidDataException("ERROR: can not found pdu=" + pdu.name);
                    }
                    RosTopicMessageConfig cfg = AssetConfigLoader.GetRosTopic(pdu.topic_message_name);
                    Type typeinfo = AssetConfigLoader.GetType(pdu.path, pdu.class_name);
                    ipdu = Activator.CreateInstance(typeinfo, topic, pdu.name, cfg.topic_message_name, cfg.topic_type_name) as IPduWriter;
                    if (pdu.conv_class_name != null)
                    {
                        IPduWriterConverter conv = AssetConfigLoader.ClassLoader(null, pdu.conv_class_name, null) as IPduWriterConverter;
                        ipdu.SetConverter(conv);
                    }
                    else
                    {
                        throw new InvalidDataException("ERROR: can not found conv_class_name=" + pdu.conv_class_name);
                    }
                }
                if (ipdu != null)
                {
                    SimpleLogger.Get().Log(Level.DEBUG, "pdu_writer: " + ipdu.GetName());
                    AssetConfigLoader.pdu_writers.Add(ipdu);
                }
            }
        }
        private static void LoadPduReaders(string filepath)
        {
            if (filepath != null)
            {
                core_config.pdu_readers = LoadJsonFile<PduReaderConfig[]>(filepath);
            }

            foreach (var pdu in core_config.pdu_readers)
            {
                IPduReader ipdu = null;
                if (pdu.topic_message_name == null)
                {
                    Pdu pdu_data = new Pdu(pdu.pdu_config_name);
                    if (pdu_data == null)
                    {
                        throw new InvalidDataException("ERROR: can not found pdu=" + pdu.name);
                    }
                    Type typeinfo = AssetConfigLoader.GetType(pdu.path, pdu.class_name);
                    ipdu = Activator.CreateInstance(typeinfo, pdu_data, pdu.name) as IPduReader;
                    SimpleLogger.Get().Log(Level.INFO, "pdu writer loaded:" + pdu.class_name);
                    if (ipdu == null)
                    {
                        throw new InvalidDataException("ERROR: can not found classname=" + pdu.class_name);
                    }
                    if (pdu.conv_class_name != null)
                    {
                        IPduReaderConverter conv = AssetConfigLoader.ClassLoader(pdu.conv_path, pdu.conv_class_name, null) as IPduReaderConverter;
                        ipdu.SetConverter(conv);
                    }
                }
                else
                {
                    Pdu topic = new Pdu(pdu.pdu_config_name);
                    if (topic == null)
                    {
                        throw new InvalidDataException("ERROR: can not found pdu=" + pdu.name);
                    }
                    SimpleLogger.Get().Log(Level.INFO, "ros topic pdu reader name = " + pdu.name);
                    RosTopicMessageConfig cfg = AssetConfigLoader.GetRosTopic(pdu.topic_message_name);
                    Type typeinfo = AssetConfigLoader.GetType(pdu.path, pdu.class_name);
                    ipdu = Activator.CreateInstance(typeinfo, topic, pdu.name, cfg.topic_message_name, cfg.topic_type_name) as IPduReader;
                    if (pdu.conv_class_name != null)
                    {
                        IPduReaderConverter conv = AssetConfigLoader.ClassLoader(null, pdu.conv_class_name, null) as IPduReaderConverter;
                        ipdu.SetConverter(conv);
                    }
                    else
                    {
                        throw new InvalidDataException("ERROR: can not found conv_class_name=" + pdu.conv_class_name);
                    }
                }
                if (ipdu != null)
                {
                    SimpleLogger.Get().Log(Level.INFO, "pdu reader name = " + ipdu.GetName());
                    AssetConfigLoader.pdu_readers.Add(ipdu);
                }
            }
        }
        private static void LoadReaderConnectors(string filepath)
        {
            if (filepath != null)
            {
                core_config.reader_connectors = LoadJsonFile<ReaderConnectorConfig[]>(filepath);
            }
            foreach (var connector in core_config.reader_connectors)
            {
                SimpleLogger.Get().Log(Level.INFO, "reader connector method_name=" + connector.method_name);
                var method = AssetConfigLoader.GetIoReader(connector.method_name);
                if (method == null)
                {
                    //throw new InvalidDataException("ERROR: can not found connector method_name=" + connector.method_name);
                    SimpleLogger.Get().Log(Level.INFO, "can not found connector method_name=" + connector.method_name);
                }
                SimpleLogger.Get().Log(Level.INFO, "reader connector pdu_name=" + connector.pdu_name);
                var pdu = AssetConfigLoader.GetIpduReader(connector.pdu_name);
                if (pdu == null)
                {
                    throw new InvalidDataException("ERROR: can not found connector pdu_name=" + connector.pdu_name);
                }
                var real_connector = ReaderConnector.Create(pdu, new ReaderChannel(method));
                real_connector.Name = connector.name;
                AssetConfigLoader.reader_connectors.Add(real_connector);
            }

        }
        private static void LoadWriterConnectors(string filepath)
        {
            if (filepath != null)
            {
                core_config.writer_connectors = LoadJsonFile<WriterConnectorConfig[]>(filepath);
            }
            foreach (var connector in core_config.writer_connectors)
            {
                var method = AssetConfigLoader.GetIoWriter(connector.method_name);
                if (method == null)
                {
                    //throw new InvalidDataException("ERROR: can not found connector method_name=" + connector.method_name);
                    SimpleLogger.Get().Log(Level.INFO, "can not found connector method_name=" + connector.method_name);
                }
                var w_pdu = AssetConfigLoader.GetIpduWriter(connector.pdu_name);
                var r_pdu = AssetConfigLoader.GetIpduReader(connector.pdu_name);
                if ((w_pdu == null) && (r_pdu == null))
                {
                    throw new InvalidDataException("ERROR: can not found connector pdu_name=" + connector.pdu_name);
                }
                if (w_pdu != null)
                {
                    var real_connector = WriterConnector.Create(w_pdu, new WriterChannel(method));
                    real_connector.Name = connector.name;
                    AssetConfigLoader.writer_connectors.Add(real_connector);
                }
                else
                {
                    var real_connector = WriterConnector.Create(r_pdu, new WriterChannel(method));
                    real_connector.Name = connector.name;
                    AssetConfigLoader.writer_connectors.Add(real_connector);
                }
            }
        }
        private static void LoadPduChannelConnectors(string filepath)
        {
            if (filepath != null)
            {
                core_config.pdu_channel_connectors = LoadJsonFile<PduChannelConnectorConfig[]>(filepath);
            }
            foreach (var connector in core_config.pdu_channel_connectors)
            {
                var real_connector = PduChannelConnector.Create(connector.outside_asset_name);
                var reader = AssetConfigLoader.GetReaderConnector(connector.reader_connector_name);
                var writer = AssetConfigLoader.GetWriterConnector(connector.writer_connector_name);
                if ((reader == null) && (writer == null))
                {
                    throw new InvalidDataException("ERROR: can not found pdu channel connector writer=" + connector.writer_connector_name + " reader=" + connector.reader_connector_name);
                }
                if (writer == null)
                {
                    SimpleLogger.Get().Log(Level.WARN, "not found writer: " + connector.writer_connector_name);
                }
                if (reader == null)
                {
                    SimpleLogger.Get().Log(Level.WARN, "not found writer: " + connector.reader_connector_name);
                }
                real_connector.Writer = writer;
                real_connector.Reader = reader;
                AssetConfigLoader.pdu_channel_connectors.Add(real_connector);
                SimpleLogger.Get().Log(Level.INFO, "PduChannelConnector :" + connector.writer_connector_name);
            }

        }
        private static void LoadInsideAssets(string filepath)
        {
            if (filepath != null)
            {
                core_config.inside_assets = LoadJsonFile<InsideAssetConfig[]>(filepath);
            }
            if (core_config.inside_assets != null)
            {
                foreach (var asset in core_config.inside_assets)
                {
                    var connector = PduIoConnector.Create(asset.name);
                    foreach (var name in asset.pdu_writer_names)
                    {
                        var pdu = AssetConfigLoader.GetIpduWriter(name);
                        if (pdu == null)
                        {
                            throw new InvalidDataException("ERROR: can not found inside asset pdu writer=" + name);
                        }
                        SimpleLogger.Get().Log(Level.DEBUG, asset.name + " :pdu io connect add writer:" + pdu.GetName());
                        connector.AddWriter(pdu);
                    }
                    foreach (var name in asset.pdu_reader_names)
                    {
                        var pdu = AssetConfigLoader.GetIpduReader(name);
                        if (pdu == null)
                        {
                            throw new InvalidDataException("ERROR: can not found inside asset pdu reader=" + name);
                        }
                        SimpleLogger.Get().Log(Level.DEBUG, asset.name + " pdu io connect add reader:" + pdu.GetName());
                        connector.AddReader(pdu);
                    }
                }
            }
        }


        private static void LoadOutsideAssets(string filepath)
        {
            if (filepath != null)
            {
                if (File.Exists(filepath))
                {
                    core_config.outside_assets = LoadJsonFile<OutsideAssetConfig[]>(filepath);
                }
            }
            if (core_config.outside_assets != null)
            {
                foreach (var asset in core_config.outside_assets)
                {
                    IOutsideAssetController controller = null;
                    if (asset.class_name != null)
                    {
                        controller = AssetConfigLoader.ClassLoader(asset.path, asset.class_name, asset.name) as IOutsideAssetController;
                    }
                    if (controller == null)
                    {
                        throw new InvalidDataException("ERROR: can not found classname=" + asset.class_name);
                    }
                    SimpleLogger.Get().Log(Level.INFO, "OutSideAsset :" + asset.name);
                    AssetConfigLoader.AddOutsideAsset(controller);
                }
            }
        }
        private static void LoadShmMethods(string filepath)
        {
            if (filepath != null)
            {
                core_config.shm_methods = LoadJsonFile<ShmMethodConfig[]>(filepath);
            }
            if (core_config.shm_methods != null)
            {
                //shm method configs
                foreach (var method in core_config.shm_methods)
                {
                    var config = new ShmConfig();
                    config.io_size = method.iosize;
                    config.asset_name = method.asset_name;
                    config.channel_id = method.channel_id;
                    if (method.is_read)
                    {
                        var real_method = new ShmReader();
                        real_method.Initialize(config);
                        real_method.Name = method.method_name;
                        AssetConfigLoader.io_readers.Add(real_method);
                    }
                    else
                    {
                        var real_method = new ShmWriter();
                        real_method.Initialize(config);
                        real_method.Name = method.method_name;
                        AssetConfigLoader.io_writers.Add(real_method);
                    }
                }
            }
        }
        private static void LoadRpcMethodsAsync(string filepath)
        {
            if (filepath != null)
            {
                core_config.rpc_methods = LoadJsonFile<RpcMethodConfig[]>(filepath);
            }
#if NO_USE_GRPC
#else
            if (core_config.rpc_methods != null)
            {
                SimpleLogger.Get().Log(Level.INFO, "START RPC CONFIG PARSE");
                //rpc method configs
                foreach (var method in core_config.rpc_methods)
                {
                    var config = new RpcConfig();
                    config.PduSize = method.pdu_size;
                    config.asset_name = method.asset_name;
                    config.channel_id = method.channel_id;
                    config.MethodType = method.method_type;
                    config.write_count = method.write_cycle;
                    if (method.is_read)
                    {
                        var real_method = new RpcReader();
                        real_method.Initialize(config);
                        real_method.Name = method.method_name;
                        AssetConfigLoader.io_readers.Add(real_method);
                    }
                    else
                    {
                        var real_method = new RpcWriter();
                        real_method.Initialize(config);
                        real_method.Name = method.method_name;
                        AssetConfigLoader.io_writers.Add(real_method);
                    }
                }
                SimpleLogger.Get().Log(Level.INFO, "END RPC CONFIG PARSE");
                SimpleLogger.Get().Log(Level.INFO, "ACTIVATE START MQTT SUBSCRIBE");
                HakoMqtt.StartSubscribe();
                SimpleLogger.Get().Log(Level.INFO, "ACTIVATE DONE MQTT SUBSCRIBE");
            }
#endif
        }
        private static void LoadWorldConfig(string filepath)
        {
            if (filepath != null)
            {
                if (File.Exists(filepath))
                {
                    core_config.param_world_config = LoadJsonFile<ParamWorldConfigContainer>(filepath);
                }
            }
        }

        private static void LoadPduOffsetConfig(string filepath)
        {
            if (filepath != null)
            {
                if (Directory.Exists(filepath))
                {
                    PduOffset.ParseAll(filepath);
                }
            }
        }

        public static void Load(string filepath)
        {
            core_config = LoadJsonFile<CoreConfig>(filepath);
            LoadPduConfig(core_config.pdu_configs_path);
            if (core_config.ros_topics_path != null)
            {
#if UNITY_IOS
                string fileNameWithoutExtension = core_config.ros_topics_path.Replace("./", "").Replace(".json", "");
                TextAsset jsonTextAsset = Resources.Load<TextAsset>(fileNameWithoutExtension);
                if (jsonTextAsset == null)
                {
                    throw new ArgumentException("can not find path=" + fileNameWithoutExtension);
                }
                string jsonString = jsonTextAsset.text;
#else
                string jsonString = File.ReadAllText(core_config.ros_topics_path);
#endif
                var container = JsonConvert.DeserializeObject<RosTopicMessageConfigContainer>(jsonString);
                core_config.ros_topics = container.fields;
            }

            if (core_config.cpp_asset_name != null)
            {
                if (core_config.cpp_mode.Equals("asset_rpc"))
                {
#if NO_USE_GRPC
                    //throw new NotSupportedException("ERROR: asset_rpc is not supported..");
                    SimpleLogger.Get().Log(Level.INFO, "DEBUG MODE");
#else
                    SimpleLogger.Get().Log(Level.INFO, "START CLIENT");
                    RpcClient.StartClient(AssetConfigLoader.core_config.core_ipaddr, AssetConfigLoader.core_config.core_portno);
#endif
                }
                else
                {
                    SimpleLogger.Get().Log(Level.INFO, "SERVER MODE");
                }
            }

            //LoadWorldConfig(core_config.param_world_config_path);
            //writer pdu configs
            LoadPduWriters(core_config.pdu_writers_path);
            //reader pdu configs
            LoadPduReaders(core_config.pdu_readers_path);

            //shm methods configs
            LoadShmMethods(core_config.shm_methods_path);
            //rpc methods configs
            LoadRpcMethodsAsync(core_config.rpc_methods_path);

            //reader connectors configs
            LoadReaderConnectors(core_config.reader_connectors_path);
            //writer connectors configs
            LoadWriterConnectors(core_config.writer_connectors_path);

            //pdu channel connectors configs
            LoadPduChannelConnectors(core_config.pdu_channel_connectors_path);
            //inside asset configs
            LoadInsideAssets(core_config.inside_assets_path);

            //outside asset configs
            //LoadOutsideAssets(core_config.outside_assets_path);

            //Pdu offset configs
            LoadPduOffsetConfig(core_config.pdu_bin_offset_package_dir);
        }
    }
}

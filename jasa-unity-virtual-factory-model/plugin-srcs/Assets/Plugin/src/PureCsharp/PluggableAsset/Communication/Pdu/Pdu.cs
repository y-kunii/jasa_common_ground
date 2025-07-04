using Hakoniwa.Core.Utils.Logger;
using System;
using System.Collections.Generic;
using System.Text;

namespace Hakoniwa.PluggableAsset.Communication.Pdu
{
    public class Pdu : IPduReadOperation, IPduWriteOperation
    {
        private PduDataFieldsConfig pdu_config;
        private string pdu_type_name = null;
        private string my_package_name = null;
        private Dictionary<string, SByte> field_int8 = new Dictionary<string, SByte>();
        private Dictionary<string, Byte> field_uint8 = new Dictionary<string, Byte>();
        private Dictionary<string, Int16> field_int16 = new Dictionary<string, Int16>();
        private Dictionary<string, UInt16> field_uint16 = new Dictionary<string, UInt16>();
        private Dictionary<string, Int32> field_int32 = new Dictionary<string, Int32>();
        private Dictionary<string, UInt32> field_uint32 = new Dictionary<string, UInt32>();
        private Dictionary<string, UInt64> field_uint64 = new Dictionary<string, UInt64>();
        private Dictionary<string, Int64> field_int64 = new Dictionary<string, Int64>();
        private Dictionary<string, float> field_float32 = new Dictionary<string, float>();
        private Dictionary<string, double> field_float64 = new Dictionary<string, double>();
        private Dictionary<string, string> field_string = new Dictionary<string, string>();
        private Dictionary<string, bool> field_bool = new Dictionary<string, bool>();
        private Dictionary<string, Pdu> field_struct = new Dictionary<string, Pdu>();

        private Dictionary<string, SByte[]> field_int8_array = new Dictionary<string, SByte[]>();
        private Dictionary<string, Byte[]> field_uint8_array = new Dictionary<string, Byte[]>();
        private Dictionary<string, Int16[]> field_int16_array = new Dictionary<string, Int16[]>();
        private Dictionary<string, UInt16[]> field_uint16_array = new Dictionary<string, UInt16[]>();
        private Dictionary<string, Int32[]> field_int32_array = new Dictionary<string, Int32[]>();
        private Dictionary<string, UInt32[]> field_uint32_array = new Dictionary<string, UInt32[]>();
        private Dictionary<string, UInt64[]> field_uint64_array = new Dictionary<string, UInt64[]>();
        private Dictionary<string, Int64[]> field_int64_array = new Dictionary<string, Int64[]>();
        private Dictionary<string, float[]> field_float32_array = new Dictionary<string, float[]>();
        private Dictionary<string, double[]> field_float64_array = new Dictionary<string, double[]>();
        private Dictionary<string, string[]> field_string_array = new Dictionary<string, string[]>();
        private Dictionary<string, bool[]> field_bool_array = new Dictionary<string, bool[]>();
        private Dictionary<string, Pdu[]> field_struct_array = new Dictionary<string, Pdu[]>();

        private static int GetArraySize(string type)
        {
            if (!type.Contains("["))
            {
                return 0;
            }
            string tmp1 = type.Split('[')[1];
            string tmp2 = tmp1.Split(']')[0];
            string value = tmp2.Trim();
            if (value.Length == 0)
            {
                //可変配列
                return 1;
            }
            return int.Parse(value);
        }
        private static bool IsPrimitiveType(string type)
        {
            switch (type)
            {
                case "int8":
                    return true;
                case "uint8":
                    return true;
                case "int16":
                    return true;
                case "uint16":
                    return true;
                case "int32":
                    return true;
                case "uint32":
                    return true;
                case "int64":
                    return true;
                case "uint64":
                    return true;
                case "float32":
                    return true;
                case "float64":
                    return true;
                case "string":
                    return true;
                case "bool":
                    return true;
                default:
                    return false;
            }
        }
        private static bool IsArray(string type)
        {
            if (type.Contains("["))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        private static string GetArrayType(string type)
        {
            if (!type.Contains("["))
            {
                return null;
            }
            return type.Split('[')[0].Trim();
        }

        public string GetName()
        {
            return this.pdu_type_name;
        }
        public string GetPackageName()
        {
            return this.my_package_name;
        }
        private static Stack<string> packageNameStack = new Stack<string>();
        private void SetPdu(string arg_pdu_type_name, string parent_package_name = null)
        {
            if (packageNameStack.Count > 0)
            {
                SimpleLogger.Get().Log(Level.DEBUG, "SetPdu(): BEFORE: package_name = " + packageNameStack.Peek());
            }
            else
            {
                SimpleLogger.Get().Log(Level.DEBUG, "SetPdu(): BEFORE: package_name = not stacked: " + arg_pdu_type_name);
            }
            if (arg_pdu_type_name.Contains('/'))
            {
                string package_name = arg_pdu_type_name.Split('/')[0];
                packageNameStack.Push(package_name);
            }
            else if (packageNameStack.Count > 0)
            {
                string package_name = packageNameStack.Peek();
                packageNameStack.Push(package_name);
            }
            SimpleLogger.Get().Log(Level.DEBUG, "SetPdu(): arg_pdu_type_name = " + arg_pdu_type_name);
            if (packageNameStack.Count > 0)
            {
                this.my_package_name = packageNameStack.Peek();
                SimpleLogger.Get().Log(Level.DEBUG, "SetPdu(): AFTER: package_name = " + this.my_package_name);
            }
            if (this.my_package_name == null)
            {
                throw new ArgumentException("Can not found package_name:" + arg_pdu_type_name);
            }
            if (this.pdu_config == null)
            {
                string tmp_type = arg_pdu_type_name;
                if (arg_pdu_type_name.Contains('/'))
                {
                    //nothing to do
                }
                else if (packageNameStack.Count > 0)
                {
                    tmp_type = packageNameStack.Peek() + "/" + arg_pdu_type_name;
                }
                var e = AssetConfigLoader.GetPduConfig(tmp_type);
                if (e == null)
                {
                    throw new ArgumentException("Can not found pdu type:" + arg_pdu_type_name);
                }
                var tmp = new PduDataFieldsConfig();
                tmp.fields = e.fields;
                this.pdu_config = tmp;
            }

            foreach (var e in pdu_config.fields)
            {
                if (IsArray(e.type))
                {
                    SimpleLogger.Get().Log(Level.DEBUG, "type " + e.type + " name " + e.name);
                    string array_type = GetArrayType(e.type);
                    if (IsPrimitiveType(array_type))
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "PRIMITIVE ARRAY:" + e.name + " type=" + e.type);
                        this.SetArrayInitValue(array_type, e.name, GetArraySize(e.type));
                    }
                    else
                    {
                        Pdu[] elms = new Pdu[GetArraySize(e.type)];
                        for (int i = 0; i < elms.Length; i++)
                        {
                            elms[i] = new Pdu(array_type);
                        }
                        SimpleLogger.Get().Log(Level.DEBUG, "STRUCT ARRAY:" + e.name + " type=" + e.type + " array_size: " + elms.Length);
                        this.field_struct_array.Add(e.name, elms);
                    }
                }
                else
                {
                    if (IsPrimitiveType(e.type))
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "PRIMITIVE:" + e.name + " type=" + e.type);
                        this.SetValue(e.type, e.name);
                    }
                    else
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "STRUCT:" + e.name + " type=" + e.type);
                        this.field_struct.Add(e.name, new Pdu(e.type));
                    }

                }
            }
            packageNameStack.Pop();
            return;
        }

        public Pdu(string arg_pdu_type_name, string parent_package_name = null)
        {
            this.pdu_type_name = arg_pdu_type_name;
            this.SetPdu(arg_pdu_type_name, parent_package_name);
        }

        public void Reset()
        {
            if (this.my_package_name == null)
            {
                throw new ArgumentException("Can not found package_name for " + this.pdu_type_name);
            }
            packageNameStack.Push(this.my_package_name);
            foreach (var e in pdu_config.fields)
            {
                if (IsArray(e.type))
                {
                    SimpleLogger.Get().Log(Level.DEBUG, "RESET type " + e.type + " name " + e.name);
                    string array_type = GetArrayType(e.type);
                    if (IsPrimitiveType(array_type))
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "RESET PRIMITIVE ARRAY:" + e.name + " type=" + e.type);
                        //this.ResetArrayInitValue(array_type, e.name, GetArraySize(e.type));
                        this.ResetArrayInitValue(array_type, e.name);
                    }
                    else
                    {
                        Pdu[] elms = new Pdu[this.field_struct_array[e.name].Length];
                        //Pdu[] elms = new Pdu[GetArraySize(e.type)];
                        for (int i = 0; i < this.field_struct_array[e.name].Length; i++)
                        {
                            elms[i] = new Pdu(array_type);
                        }
                        SimpleLogger.Get().Log(Level.DEBUG, "RESET STRUCT ARRAY:" + e.name + " type=" + e.type);
                        this.field_struct_array.Remove(e.name);
                        this.field_struct_array.Add(e.name, elms);
                    }
                }
                else
                {
                    if (IsPrimitiveType(e.type))
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "RESET PRIMITIVE:" + e.name + " type=" + e.type);
                        this.ResetValue(e.type, e.name);
                    }
                    else
                    {
                        SimpleLogger.Get().Log(Level.DEBUG, "RESET STRUCT:" + e.name + " type=" + e.type);
                        this.field_struct.Remove(e.name);
                        this.field_struct.Add(e.name, new Pdu(e.type));
                    }

                }
            }
        }

        public Pdu(string arg_pdu_type_name, PduDataFieldsConfig config)
        {
            this.pdu_type_name = arg_pdu_type_name;
            this.pdu_config = config;
            this.SetPdu(arg_pdu_type_name);
        }

        private void SetValue(string type, string name)
        {
            switch (type)
            {
                case "int8":
                    this.field_int8.Add(name, 0);
                    break;
                case "uint8":
                    this.field_uint8.Add(name, 0);
                    break;
                case "int16":
                    this.field_int16.Add(name, 0);
                    break;
                case "uint16":
                    this.field_uint16.Add(name, 0);
                    break;
                case "int32":
                    this.field_int32.Add(name, 0);
                    break;
                case "uint32":
                    this.field_uint32.Add(name, 0);
                    break;
                case "int64":
                    this.field_int64.Add(name, 0);
                    break;
                case "uint64":
                    this.field_uint64.Add(name, 0);
                    break;
                case "float32":
                    this.field_float32.Add(name, 0);
                    break;
                case "float64":
                    this.field_float64.Add(name, 0);
                    break;
                case "string":
                    this.field_string.Add(name, "");
                    break;
                case "bool":
                    this.field_bool.Add(name, false);
                    break;
                default:
                    break;
            }
        }
        private void ResetValue(string type, string name)
        {
            switch (type)
            {
                case "int8":
                    this.field_int8.Remove(name);
                    this.field_int8.Add(name, 0);
                    break;
                case "uint8":
                    this.field_uint8.Remove(name);
                    this.field_uint8.Add(name, 0);
                    break;
                case "int16":
                    this.field_int16.Remove(name);
                    this.field_int16.Add(name, 0);
                    break;
                case "uint16":
                    this.field_uint16.Remove(name);
                    this.field_uint16.Add(name, 0);
                    break;
                case "int32":
                    this.field_int32.Remove(name);
                    this.field_int32.Add(name, 0);
                    break;
                case "uint32":
                    this.field_uint32.Remove(name);
                    this.field_uint32.Add(name, 0);
                    break;
                case "int64":
                    this.field_int64.Remove(name);
                    this.field_int64.Add(name, 0);
                    break;
                case "uint64":
                    this.field_uint64.Remove(name);
                    this.field_uint64.Add(name, 0);
                    break;
                case "float32":
                    this.field_float32.Remove(name);
                    this.field_float32.Add(name, 0);
                    break;
                case "float64":
                    this.field_float64.Remove(name);
                    this.field_float64.Add(name, 0);
                    break;
                case "string":
                    this.field_string.Remove(name);
                    this.field_string.Add(name, "");
                    break;
                case "bool":
                    this.field_bool.Remove(name);
                    this.field_bool.Add(name, false);
                    break;
                default:
                    break;
            }
        }
        private void SetArrayInitValue(string array_type, string name, int array_size)
        {
            switch (array_type)
            {
                case "int8":
                    this.field_int8_array.Add(name, new sbyte[array_size]);
                    break;
                case "uint8":
                    this.field_uint8_array.Add(name, new byte[array_size]);
                    break;
                case "int16":
                    this.field_int16_array.Add(name, new Int16[array_size]);
                    break;
                case "uint16":
                    this.field_uint16_array.Add(name, new UInt16[array_size]);
                    break;
                case "int32":
                    this.field_int32_array.Add(name, new Int32[array_size]);
                    break;
                case "uint32":
                    this.field_uint32_array.Add(name, new UInt32[array_size]);
                    break;
                case "int64":
                    this.field_int64_array.Add(name, new Int64[array_size]);
                    break;
                case "uint64":
                    this.field_uint64_array.Add(name, new UInt64[array_size]);
                    break;
                case "float32":
                    this.field_float32_array.Add(name, new float[array_size]);
                    break;
                case "float64":
                    this.field_float64_array.Add(name, new double[array_size]);
                    break;
                case "string":
                    this.field_string_array.Add(name, new string[array_size]);
                    break;
                case "bool":
                    this.field_bool_array.Add(name, new bool[array_size]);
                    break;
                default:
                    break;
            }
        }
        private void ResetArrayInitValue(string array_type, string name)
        {
            int array_size = 0;
            switch (array_type)
            {
                case "int8":
                    array_size = this.field_int8_array[name].Length;
                    this.field_int8_array.Remove(name);
                    this.field_int8_array.Add(name, new sbyte[array_size]);
                    break;
                case "uint8":
                    array_size = this.field_uint8_array[name].Length;
                    this.field_uint8_array.Remove(name);
                    this.field_uint8_array.Add(name, new byte[array_size]);
                    break;
                case "int16":
                    array_size = this.field_int16_array[name].Length;
                    this.field_int16_array.Remove(name);
                    this.field_int16_array.Add(name, new Int16[array_size]);
                    break;
                case "uint16":
                    array_size = this.field_uint16_array[name].Length;
                    this.field_uint16_array.Remove(name);
                    this.field_uint16_array.Add(name, new UInt16[array_size]);
                    break;
                case "int32":
                    array_size = this.field_int32_array[name].Length;
                    this.field_int32_array.Remove(name);
                    this.field_int32_array.Add(name, new Int32[array_size]);
                    break;
                case "uint32":
                    array_size = this.field_uint32_array[name].Length;
                    this.field_uint32_array.Remove(name);
                    this.field_uint32_array.Add(name, new UInt32[array_size]);
                    break;
                case "int64":
                    array_size = this.field_int64_array[name].Length;
                    this.field_int64_array.Remove(name);
                    this.field_int64_array.Add(name, new Int64[array_size]);
                    break;
                case "uint64":
                    array_size = this.field_uint64_array[name].Length;
                    this.field_uint64_array.Remove(name);
                    this.field_uint64_array.Add(name, new UInt64[array_size]);
                    break;
                case "float32":
                    array_size = this.field_float32_array[name].Length;
                    this.field_float32_array.Remove(name);
                    this.field_float32_array.Add(name, new float[array_size]);
                    break;
                case "float64":
                    array_size = this.field_float64_array[name].Length;
                    this.field_float64_array.Remove(name);
                    this.field_float64_array.Add(name, new double[array_size]);
                    break;
                case "string":
#if true
                    array_size = this.field_string_array[name].Length;
                    this.field_string_array.Remove(name);
                    this.field_string_array.Add(name, new string[array_size]);
#else
                    //original code...
                    array_size = this.field_string_array[name].Length;
                    if (this.field_string_array.Count == 0) {
                    	this.field_string_array.Remove(name);
                    	this.field_string_array.Add(name, new string[array_size]);
		            }
#endif
                    break;
                case "bool":
                    array_size = this.field_bool_array[name].Length;
                    this.field_bool_array.Remove(name);
                    this.field_bool_array.Add(name, new bool[array_size]);
                    break;
                default:
                    break;
            }
        }

        public IPduReadOperation GetPduReadOps()
        {
            return this;
        }
        public IPduWriteOperation GetPduWriteOps()
        {
            return this;
        }


        public void SetData(string field_name, byte value)
        {
            if (!this.field_uint8.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint8[field_name] = value;
        }

        public void SetData(string field_name, sbyte value)
        {
            if (!this.field_int8.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int8[field_name] = value;
        }

        public void SetData(string field_name, ushort value)
        {
            if (!this.field_uint16.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint16[field_name] = value;
        }

        public void SetData(string field_name, short value)
        {
            if (!this.field_int16.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int16[field_name] = value;
        }

        public void SetData(string field_name, uint value)
        {
            if (!this.field_uint32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            //Debug.Log("SetData: " + this.GetHashCode() + "field_name " + field_name + " value=" + value);
            //SimpleLogger.Get().Log(Level.DEBUG, "uint32 name= " + field_name + " data=" + value);
            this.field_uint32[field_name] = value;
        }

        public void SetData(string field_name, int value)
        {
            if (!this.field_int32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            //SimpleLogger.Get().Log(Level.DEBUG, "int32 name= " + field_name + " data=" + value);
            this.field_int32[field_name] = value;
        }

        public void SetData(string field_name, ulong value)
        {
            if (!this.field_uint64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint64[field_name] = value;
        }

        public void SetData(string field_name, long value)
        {
            if (!this.field_int64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int64[field_name] = value;
        }

        public void SetData(string field_name, double value)
        {
            if (!this.field_float64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float64[field_name] = value;
        }

        public void SetData(string field_name, float value)
        {
            if (!this.field_float32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float32[field_name] = value;
        }

        public void SetData(string field_name, string value)
        {
            if (!this.field_string.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            //SimpleLogger.Get().Log(Level.DEBUG, "pdu_name=" + this.GetName() + " value=" + value);
            this.field_string[field_name] = value;
        }

        public void SetData(string field_name, bool value)
        {
            if (!this.field_bool.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            //SimpleLogger.Get().Log(Level.DEBUG, "pdu_name=" + this.GetName() + " value=" + value);
            this.field_bool[field_name] = value;
        }

        public void SetData(string field_name, byte[] value)
        {
            if (!this.field_uint8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint8_array[field_name] = value;
        }
        public void SetData(string field_name, int off, byte value)
        {
            if (!this.field_uint8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint8_array[field_name][off] = value;
        }

        public void SetData(string field_name, sbyte[] value)
        {
            if (!this.field_int8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int8_array[field_name] = value;
        }
        public void SetData(string field_name, int off, sbyte value)
        {
            if (!this.field_int8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int8_array[field_name][off] = value;
        }

        public void SetData(string field_name, ushort[] value)
        {
            if (!this.field_uint16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint16_array[field_name] = value;
        }
        public void SetData(string field_name, int off, ushort value)
        {
            if (!this.field_uint16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint16_array[field_name][off] = value;
        }

        public void SetData(string field_name, short[] value)
        {
            if (!this.field_int16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int16_array[field_name] = value;
        }
        public void SetData(string field_name, int off, short value)
        {
            if (!this.field_int16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int16_array[field_name][off] = value;
        }

        public void SetData(string field_name, uint[] value)
        {
            if (!this.field_uint32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint32_array[field_name] = value;
        }
        public void SetData(string field_name, int off, uint value)
        {
            if (!this.field_uint32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint32_array[field_name][off] = value;
        }

        public void SetData(string field_name, int[] value)
        {
            if (!this.field_int32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int32_array[field_name] = value;
        }
        public void SetData(string field_name, int off, int value)
        {
            if (!this.field_int32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int32_array[field_name][off] = value;
        }

        public void SetData(string field_name, ulong[] value)
        {
            if (!this.field_uint64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint64_array[field_name] = value;
        }
        public void SetData(string field_name, int off, ulong value)
        {
            if (!this.field_uint64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_uint64_array[field_name][off] = value;
        }

        public void SetData(string field_name, long[] value)
        {
            if (!this.field_int64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int64_array[field_name] = value;
        }
        public void SetData(string field_name, int off, long value)
        {
            if (!this.field_int64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_int64_array[field_name][off] = value;
        }

        public void SetData(string field_name, double[] value)
        {
            if (!this.field_float64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float64_array[field_name] = value;
        }
        public void SetData(string field_name, int off, double value)
        {
            if (!this.field_float64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float64_array[field_name][off] = value;
        }

        public void SetData(string field_name, float[] value)
        {
            if (!this.field_float32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float32_array[field_name] = value;
        }
        public void SetData(string field_name, int off, float value)
        {
            if (!this.field_float32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_float32_array[field_name][off] = value;
        }

        public void SetData(string field_name, string[] value)
        {
            if (!this.field_string_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_string_array[field_name] = value;
        }
        public void SetData(string field_name, int off, string value)
        {
            if (!this.field_string_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_string_array[field_name][off] = value;
        }

        public void SetData(string field_name, bool[] value)
        {
            if (!this.field_bool_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_bool_array[field_name] = value;
        }
        public void SetData(string field_name, int off, bool value)
        {
            if (!this.field_bool_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name + " value=" + value);
            }
            this.field_bool_array[field_name][off] = value;
        }

        public void SetData(string field_name, Pdu pdu)
        {
            if (!this.field_struct.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            //Debug.Log("SetData PDU: field_name=" + field_name);
            this.field_struct[field_name] = pdu;
        }

        public void SetData(string field_name, Pdu[] pdu)
        {
            if (!this.field_struct_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            //Debug.Log("SetData PDU[]: field_name=" + field_name);
            this.field_struct_array[field_name] = pdu;
        }
        public void SetData(string field_name, int off, Pdu pdu)
        {
            if (!this.field_struct_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            this.field_struct_array[field_name][off] = pdu;
        }


        public sbyte GetDataInt8(string field_name)
        {
            if (!this.field_int8.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int8[field_name];
        }

        public byte GetDataUInt8(string field_name)
        {
            if (!this.field_uint8.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint8[field_name];
        }

        public short GetDataInt16(string field_name)
        {
            if (!this.field_int16.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int16[field_name];
        }

        public ushort GetDataUInt16(string field_name)
        {
            if (!this.field_uint16.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint16[field_name];
        }

        public int GetDataInt32(string field_name)
        {
            if (!this.field_int32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int32[field_name];
        }

        public uint GetDataUInt32(string field_name)
        {
            if (!this.field_uint32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            //Debug.Log("GetDataUInt32: " + this.GetHashCode() + ": " + field_name + " :" + field_uint32[field_name]);
            return field_uint32[field_name];
        }

        public ulong GetDataUInt64(string field_name)
        {
            if (!this.field_uint64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint64[field_name];
        }

        public long GetDataInt64(string field_name)
        {
            if (!this.field_int64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int64[field_name];
        }

        public float GetDataFloat32(string field_name)
        {
            if (!this.field_float32.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_float32[field_name];
        }

        public double GetDataFloat64(string field_name)
        {
            if (!this.field_float64.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_float64[field_name];
        }

        public string GetDataString(string field_name)
        {
            if (!this.field_string.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_string[field_name];
        }

        public bool GetDataBool(string field_name)
        {
            if (!this.field_bool.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_bool[field_name];
        }

        public byte[] GetDataBytes(string field_name)
        {
            throw new NotImplementedException();
        }

        public sbyte[] GetDataInt8Array(string field_name)
        {
            if (!this.field_int8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int8_array[field_name];
        }

        public byte[] GetDataUInt8Array(string field_name)
        {
            if (!this.field_uint8_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint8_array[field_name];
        }

        public short[] GetDataInt16Array(string field_name)
        {
            if (!this.field_int16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int16_array[field_name];
        }

        public ushort[] GetDataUInt16Array(string field_name)
        {
            if (!this.field_uint16_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint16_array[field_name];
        }

        public int[] GetDataInt32Array(string field_name)
        {
            if (!this.field_int32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int32_array[field_name];
        }

        public uint[] GetDataUInt32Array(string field_name)
        {
            if (!this.field_uint32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint32_array[field_name];
        }

        public ulong[] GetDataUInt64Array(string field_name)
        {
            if (!this.field_uint64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_uint64_array[field_name];
        }

        public long[] GetDataInt64Array(string field_name)
        {
            if (!this.field_int64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_int64_array[field_name];
        }

        public float[] GetDataFloat32Array(string field_name)
        {
            if (!this.field_float32_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_float32_array[field_name];
        }

        public double[] GetDataFloat64Array(string field_name)
        {
            if (!this.field_float64_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_float64_array[field_name];
        }

        public string[] GetDataStringArray(string field_name)
        {
            if (!this.field_string_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_string_array[field_name];
        }

        public bool[] GetDataBoolArray(string field_name)
        {
            if (!this.field_bool_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_bool_array[field_name];
        }

        public Pdu Ref(string field_name)
        {
            if (field_name == null)
            {
                return this;
            }
            if (!this.field_struct.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_struct[field_name];
        }

        public Pdu[] Refs(string field_name)
        {
            if (!this.field_struct_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            return field_struct_array[field_name];
        }

        public void InitializePduArray(string field_name, int array_size)
        {
            if (!this.field_struct_array.ContainsKey(field_name))
            {
                throw new ArgumentException("Invalid PDU access : field_name=" + field_name);
            }
            Pdu org = field_struct_array[field_name][0];
            field_struct_array[field_name] = new Pdu[array_size];
            for (int i = 0; i < array_size; i++)
            {
                field_struct_array[field_name][i] = new Pdu(org.pdu_type_name);
            }
            SimpleLogger.Get().Log(Level.DEBUG, "Initialize STRUCT ARRAY:" + field_name + " type=" + org.pdu_type_name);
        }
    }
}

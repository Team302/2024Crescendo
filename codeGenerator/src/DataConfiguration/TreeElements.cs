using System;
using System.ComponentModel;
using System.Reflection;
using System.Xml.Serialization;
using Configuration;

namespace DataConfiguration
{
    // =================================== Rules =====================================
    // A property named __units__ will be converted to the list of physical units
    // A property named value__ will not be shown in the tree directly. Its value is shown in the parent node
    // Attributes are only allowed on the standard types (uint, int, double, bool) and on doubleParameter, unitParameter, intParameter, boolParameter
    // The attribute PhysicalUnitsFamily can only be applied on doubleParameter, uintParameter, intParameter, boolParameter
    //
    // 

    [Serializable]
    abstract public class baseElement
    {
        [XmlIgnore]
        public object parent { get; set; } = null;
        [XmlIgnore]
        public bool isConstant { get; set; } = false; // Defined by an attribute
        [XmlIgnore]
        public bool isTunable { get; set; } = false; // Defined by an attribute
        public physicalUnit.Family unitsFamily { get; set; } = physicalUnit.Family.none; // Defined by an attribute or through the GUI
        [XmlIgnore]
        public bool unitsFamilyDefinedByAttribute { get; set; } = false; // true if an attribute defines the unitsFamily
        [XmlIgnore]
        public string physicalUnits { get; set; } = ""; // Defined by the GUI
        [XmlIgnore]
        public valueRange range { get; set; } // Defined by an attribute
        [XmlIgnore]
        public defaultValue theDefault { get; set; } // Defined by an attribute
        [XmlIgnore]
        public string description { get; set; } = "";
        [XmlIgnore]
        public bool showExpanded { get; set; } = true;

        public string name { get; set; }

        public baseElement()
        {
            range = new valueRange();
            theDefault = new defaultValue();
        }
    }

    [Serializable]
    public class valueRange
    {
        public double minRange { get; set; } = Int32.MinValue;
        public double maxRange { get; set; } = Int32.MaxValue;
    }

    [Serializable]
    public class defaultValue
    {
        public object value { get; set; } = null;
    }


    [Serializable()]
    [NotUserAddable]
    [XmlInclude(typeof(parameter))]
    [XmlInclude(typeof(uintParameter))]
    [XmlInclude(typeof(intParameter))]
    [XmlInclude(typeof(doubleParameter))]
    [XmlInclude(typeof(boolParameter))]

    public partial class parameter : baseElement
    {
        
        public string __units__ { get; set; } = "";

        [Constant()]
        public string type { get; set; }
        public parameter()
        {
            name = GetType().Name;
            helperFunctions.initializeDefaultValues(this);
        }

        virtual public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (propertyName == "name")
                return string.Format("{0} ({1})", propertyName, name);
            else if (propertyName == "type")
                return string.Format("{0} ({1})", propertyName, type);

            return null;
        }
    }

    //#region string parameter
    //[Serializable()]
    //[NotUserAddable]
    //public partial class stringParameter : parameter
    //{
    //    [DefaultValue(0u)]
    //    public string value__ { get; set; }

    //    public stringParameter()
    //    {
    //        type = value__.GetType().Name;
    //    }
    //    override public string getDisplayName(string instanceName, out helperFunctions.RefreshLevel refresh)
    //    {
    //        refresh = helperFunctions.RefreshLevel.parentHeader;

    //        return string.Format("{0} ({1})", instanceName, value__);
    //    }
    //}
    //#endregion

    #region double definitions
    [Serializable()]
    [NotUserAddable]
    public partial class doubleParameter : parameter
    {
        public double value { get; set; }

        public doubleParameter()
        {
            range.minRange = Convert.ToDouble(decimal.MinValue/2);
            range.maxRange = Convert.ToDouble(decimal.MaxValue/2);
            showExpanded = false;
            type = value.GetType().Name;
        }
        override public string getDisplayName(string instanceName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;

            if (string.IsNullOrEmpty(__units__))
                return string.Format("{0} ({1})", instanceName, value);
            else
                return string.Format("{0} ({1} {2})", instanceName, value, __units__);
        }
    }

    [Serializable()]
    public partial class doubleParameterUserDefinedBase : parameter
    {
        protected string getDisplayName(string propertyName, double value, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return string.Format("{0} ({1} {2})", name, value, __units__);
            else if (propertyName == "value")
            {
                refresh = helperFunctions.RefreshLevel.parentHeader;
                return string.Format("value ({0} {1})", value, __units__);
            }
            else if (propertyName == "unitsFamily")
            {
                refresh = helperFunctions.RefreshLevel.fullParent;
                return string.Format("{0} ({1})", propertyName, unitsFamily);
            }

            return base.getDisplayName(propertyName, out refresh);
        }
    }

    [Serializable()]
    public partial class doubleParameterUserDefinedNonTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        public double value { get; set; }
        public doubleParameterUserDefinedNonTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }

    [Serializable()]
    public partial class doubleParameterUserDefinedTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        [TunableParameter()]
        public double value { get; set; }
        public doubleParameterUserDefinedTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }
    #endregion

    #region int definitions
    [Serializable()]
    [NotUserAddable]
    public partial class intParameter : parameter
    {
        public int value { get; set; }

        public intParameter()
        {
            range.minRange = Convert.ToDouble(int.MinValue);
            range.maxRange = Convert.ToDouble(int.MaxValue);
            showExpanded = false;
            type = value.GetType().Name;
        }
        override public string getDisplayName(string instanceName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;

            if (string.IsNullOrEmpty(__units__))
                return string.Format("{0} ({1})", instanceName, value);
            else
                return string.Format("{0} ({1} {2})", instanceName, value, __units__);
        }
    }

    [Serializable()]
    public partial class intParameterUserDefinedBase : parameter
    {
        protected string getDisplayName(string propertyName, double value, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return string.Format("{0} ({1} {2})", name, value, __units__);
            else if (propertyName == "value")
            {
                refresh = helperFunctions.RefreshLevel.parentHeader;
                return string.Format("value ({0} {1})", value, __units__);
            }
            else if (propertyName == "unitsFamily")
            {
                refresh = helperFunctions.RefreshLevel.fullParent;
                return string.Format("{0} ({1})", propertyName, unitsFamily);
            }

            return base.getDisplayName(propertyName, out refresh);
        }
    }

    [Serializable()]
    public partial class intParameterUserDefinedNonTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        public int value { get; set; } = 0;
        public intParameterUserDefinedNonTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }

    [Serializable()]
    public partial class intParameterUserDefinedTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        [TunableParameter()]
        public int value { get; set; } = 0;
        public intParameterUserDefinedTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }
    #endregion

    #region uint definitions
    [Serializable()]
    [NotUserAddable]
    public partial class uintParameter : parameter
    {
        public uint value { get; set; }

        public uintParameter()
        {
            range.minRange = Convert.ToDouble(uint.MinValue);
            range.maxRange = Convert.ToDouble(uint.MaxValue);
            showExpanded = false;
            type = value.GetType().Name;
        }
        override public string getDisplayName(string instanceName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;

            if (string.IsNullOrEmpty(__units__))
                return string.Format("{0} ({1})", instanceName, value);
            else
                return string.Format("{0} ({1} {2})", instanceName, value, __units__);
        }
    }

    [Serializable()]
    public partial class uintParameterUserDefinedBase : parameter
    {

        protected string getDisplayName(string propertyName, double value, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return string.Format("{0} ({1} {2})", name, value, __units__);
            else if (propertyName == "value")
            {
                refresh = helperFunctions.RefreshLevel.parentHeader;
                return string.Format("value ({0} {1})", value, __units__);
            }
            else if (propertyName == "unitsFamily")
            {
                refresh = helperFunctions.RefreshLevel.fullParent;
                return string.Format("{0} ({1})", propertyName, unitsFamily);
            }

            return base.getDisplayName(propertyName, out refresh);
        }
    }

    [Serializable()]
    public partial class uintParameterUserDefinedNonTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        public int value { get; set; } = 0;
        public uintParameterUserDefinedNonTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }

    [Serializable()]
    public partial class uintParameterUserDefinedTunable : doubleParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        [TunableParameter()]
        public int value { get; set; } = 0;
        public uintParameterUserDefinedTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }
    #endregion

    #region bool definitions
    [Serializable()]
    [NotUserAddable]
    public partial class boolParameter : parameter
    {
        public bool value { get; set; }

        public boolParameter()
        {
            showExpanded = false;
            type = value.GetType().Name;
        }
        override public string getDisplayName(string instanceName, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.parentHeader;

            return string.Format("{0} ({1})", instanceName, value);
        }
    }

    [Serializable()]
    public partial class boolParameterUserDefinedBase : parameter
    {
        protected string getDisplayName(string propertyName, bool value, out helperFunctions.RefreshLevel refresh)
        {
            refresh = helperFunctions.RefreshLevel.none;

            if (string.IsNullOrEmpty(propertyName))
                return string.Format("{0} ({1})", name, value);
            else if (propertyName == "value")
            {
                refresh = helperFunctions.RefreshLevel.parentHeader;
                return string.Format("value ({0})", value);
            }

            return base.getDisplayName(propertyName, out refresh);
        }
    }

    [Serializable()]
    public partial class boolParameterUserDefinedNonTunable : boolParameterUserDefinedBase
    {
        [DefaultValue(false)]
        public bool value { get; set; } = false;
        public boolParameterUserDefinedNonTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }

    [Serializable()]
    public partial class boolParameterUserDefinedTunable : boolParameterUserDefinedBase
    {
        [DefaultValue(0u)]
        [TunableParameter()]
        public bool value { get; set; } = false;
        public boolParameterUserDefinedTunable()
        {
            type = value.GetType().Name;
        }

        override public string getDisplayName(string propertyName, out helperFunctions.RefreshLevel refresh)
        {
            return getDisplayName(propertyName, value, out refresh);
        }
    }
    #endregion

    #region ====================== Attribute definitions
    // if applied to a property, it means that live tuning over network tables is enabled
    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field | AttributeTargets.Class, AllowMultiple = false)]
    public class TunableParameterAttribute : Attribute
    {
        public TunableParameterAttribute()
        {
        }
    }

    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, AllowMultiple = false)]
    public class ConstantAttribute : Attribute
    {
        public ConstantAttribute()
        {
        }
    }

    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, AllowMultiple = false)]
    public class ConstantInMechInstanceAttribute : Attribute
    {
        public ConstantInMechInstanceAttribute()
        {
        }
    }

    [AttributeUsage(AttributeTargets.Class, AllowMultiple = false)]
    public class NotUserAddableAttribute : Attribute
    {
        public NotUserAddableAttribute()
        {
        }
    }

    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, AllowMultiple = false)]
    public class DataDescriptionAttribute : Attribute
    {
        public string description { get; set; }
        public DataDescriptionAttribute(string description)
        {
            this.description = description;
        }
    }

    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, AllowMultiple = false)]
    public class PhysicalUnitsFamilyAttribute : Attribute
    {
        public physicalUnit.Family family { get; set; }
        public PhysicalUnitsFamilyAttribute(physicalUnit.Family family)
        {
            this.family = family;
        }
    }
    #endregion

    public static class helperFunctions
    {
        public enum RefreshLevel { none, parentHeader, fullParent }
        static public void initializeDefaultValues(object obj)
        {
            PropertyInfo[] PIs = obj.GetType().GetProperties();
            foreach (PropertyInfo pi in PIs)
            {
                DefaultValueAttribute dva = pi.GetCustomAttribute<DefaultValueAttribute>();
                if (dva != null)
                {
                    object nestedObj = pi.GetValue(obj);
                    if (nestedObj != null)
                    {
                        PropertyInfo piValue__ = nestedObj.GetType().GetProperty("value");
                        if (piValue__ != null)
                            piValue__.SetValue(nestedObj, Convert.ChangeType(dva.Value, piValue__.PropertyType));
                        else
                            pi.SetValue(obj, Convert.ChangeType(dva.Value, pi.PropertyType));
                        initializeDefaultValues(nestedObj);
                    }
                    else //if (pi.PropertyType.FullName.StartsWith("System."))
                        pi.SetValue(obj, Convert.ChangeType(dva.Value, pi.PropertyType));



                    //if (pi.PropertyType.FullName.StartsWith("System."))
                    //    pi.SetValue(obj, Convert.ChangeType(dva.Value, pi.PropertyType));
                    //else
                    //{
                    //    object nestedObj = pi.GetValue(obj);
                    //    if(nestedObj != null)
                    //    {
                    //        PropertyInfo piValue__ = nestedObj.GetType().GetProperty("value__");
                    //        if (piValue__ != null)
                    //        {
                    //            piValue__.SetValue(nestedObj, Convert.ChangeType(dva.Value, piValue__.PropertyType));
                    //        }
                    //        initializeDefaultValues(nestedObj);
                    //    }
                    //    //if (nestedObj == null)
                    //    //{
                    //    //    nestedObj = Activator.CreateInstance(pi.PropertyType);
                    //    //    pi.SetValue(obj, nestedObj);
                    //    //}
                    //    //initializeDefaultValues(nestedObj);
                    //}
                }
                //else
                //{
                //    if (DataConfiguration.baseDataConfiguration.isACollection(pi.PropertyType))
                //    {
                //        object nestedObj = pi.GetValue(obj);

                //        if (nestedObj == null)
                //        {
                //            Type elementType = pi.PropertyType.GetGenericArguments().Single();
                //            nestedObj = Activator.CreateInstance(elementType);
                //            pi.SetValue(obj, nestedObj);
                //        }
                //    }
                //    else
                //    {
                //        object nestedObj = pi.GetValue(obj);
                //        if (nestedObj == null)
                //        {
                //            nestedObj = Activator.CreateInstance(pi.PropertyType);
                //            pi.SetValue(obj, nestedObj);
                //        }
                //        initializeDefaultValues(nestedObj);
                //    }
                //}
            }
        }

        public static void initializeNullProperties(object obj)
        {
            initializeNullProperties(obj, false);
        }
        public static void initializeNullProperties(object obj, bool recursive)
        {
            PropertyInfo[] propertyInfos = obj.GetType().GetProperties();

            for (int i = 0; i < propertyInfos.Length; i++)
            {
                PropertyInfo pi = propertyInfos[i];
                if (pi.PropertyType != typeof(System.String))
                {
                    object theObj = pi.GetValue(obj);
                    if (theObj == null)
                    {
                        theObj = Activator.CreateInstance(pi.PropertyType);
                        pi.SetValue(obj, theObj);
                        if (recursive)
                            initializeNullProperties(theObj);
                    }
                }
            }
        }
    }
}


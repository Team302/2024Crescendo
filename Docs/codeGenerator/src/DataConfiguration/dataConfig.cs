using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using System.Xml.Serialization;
using System.IO;
using System.Reflection;
using System.Collections;
using System.Runtime.Serialization.Formatters.Binary;
using Configuration;

namespace DataConfiguration
{
    public class baseDataConfiguration : baseReportingClass
    {
        public List<string> collectionBaseTypes = new List<string>(); 
        public List<physicalUnit> physicalUnits = new List<physicalUnit>();

        static public bool isACollection(object obj)
        {
            return isACollection(obj.GetType());
        }

        static public bool isACollection(Type t)
        {

            bool isaList = (t.Name == "List`1") && (t.Namespace == "System.Collections.Generic");
            bool isaCollection = (t.Name == "Collection`1") && (t.Namespace == "System.Collections.ObjectModel");
            return (isaCollection || isaList);
        }

        static public bool isACollectionOfType(Type theCollection, Type t)
        {
            if (isACollection(theCollection))
            {
                Type elementType = theCollection.GetGenericArguments().Single();
                return t.FullName == elementType.FullName;
            }

            return false;
        }

        public bool isASubClassedCollection(object obj)
        {
            return isASubClassedCollection(obj.GetType());
        }


        public bool isASubClassedCollection(Type t)
        {
            if (isACollection(t))
            {
                Type elementType = t.GetGenericArguments().Single();
                return collectionBaseTypes.Contains(elementType.FullName);
            }

            return false;
        }

        public bool isDerivedFromGenericClass(Type t)
        {
            if (t.BaseType != null)
            {
                return collectionBaseTypes.Contains(t.BaseType.FullName);
            }

            return false;
        }

        virtual public void load(string configFullPathFileName)
        {
        }

        virtual public void save(string theRobotConfigFullPathFileName)
        {

        }

    }

    public class baseReportingClass
    {
        public delegate void showMessage(string message);

        protected showMessage progressCallback;
        protected void addProgress(string info)
        {
            if (progressCallback != null)
                progressCallback(info);
        }
        public void setProgressCallback(showMessage callback)
        {
            progressCallback = callback;
        }
    }
}

using NetworkTables;
using NetworkTables.Tables;
using System.Collections.ObjectModel;
using System.Drawing;
using System.Windows.Forms;

namespace NetworkTablesUtils
{
    public class NTViewer
    {
        NetworkTable table;
        Button button;
        TreeView tree;
        Collection<TreeNode> treeNodes = new Collection<TreeNode>();
        bool hasAttachedListener = false;
        bool hasConnected = false;

        private static Action<ITable, string, Value, NotifyFlags> onSubTableCreation;
        private static Action<ITable, string, Value, NotifyFlags> onTableChange;

        public NTViewer(Button buton)
        {
            button = buton;

            NetworkTable.SetClientMode();
            NetworkTable.SetIPAddress("localhost"); //may have to change this to team number later?
            NetworkTable.SetPort(57231);

            Action<IRemote, ConnectionInfo, bool> onConnect = (iRemote, connectInfo, b) =>
            {
                if (b) //connected
                {
                    if (!hasConnected)
                    {
                        table = NetworkTable.GetTable("");

                        //AddListeners(table);

                        hasConnected = true;
                    }

                    button.BackColor = Color.FromArgb(100, Color.Green);
                }
                else //disconnected
                {
                    hasConnected = false;
                    button.BackColor = Color.FromArgb(100, Color.Red);
                }
            };

            NetworkTable.AddGlobalConnectionListener(onConnect, true);

            /*
            tree = treeview;

            onSubTableCreation = (table, key, value, flag) =>
            {
                tree.BeginInvoke(new Action(() =>
                {
                    if(flag == NotifyFlags.NotifyNew)
                    {
                        //string tableName = ((NetworkTable)table).ToString();
                        //Debug.WriteLine("Table Name: " + tableName);
                    }
                }));
            };

            onTableChange = (table, key, value, flag) =>
            {
                GetNodeFromTree(GetTableName(table), key);

                switch(flag)
                {
                    case NotifyFlags.NotifyNew:
                        AddToTree(table, key, value);
                        break;
                    case NotifyFlags.NotifyUpdate:
                        UpdateNodeInTree(table, key, value);
                        break;
                    default:
                        //No-op for delete at the moment
                        break;
                }
            };*/


        }
        public void ConnectToNetworkTables()
        {
            //if (!hasConnected)
            //{
            NetworkTable.Initialize();
            //}
        }


        public void PushValue(string value, string targetNTKey)
        {
            if (table != null)
                table.PutString(targetNTKey, value);
        }

        public void PushValue(double value, string targetNTKey)
        {
            if (table != null)
                table.PutNumber(targetNTKey, value);
        }

        public void PushValue(bool value, string targetNTKey)
        {
            if (table != null)
                table.PutBoolean(targetNTKey, value);
        }

        public static string ConvertFullNameToTuningKey(string fullName)
        {
            string cleanedPath = "";

            if (fullName.Contains("mechanismInstances"))
            {
                cleanedPath = fullName.Substring(fullName.IndexOf("mechanismInstances") + "mechanismInstances".Length + 1); //+1 is to get rid of leading '\'
            }

            //replace slashes for network tables
            cleanedPath = cleanedPath.Replace('\\', '/');

            //after replacing slashes, finally need to remove value from name
            cleanedPath = cleanedPath.Remove(cleanedPath.IndexOf('(') - 1); //- 1 is to get rid of space before parentheses

            return cleanedPath;
        }

        private string GetTableName(ITable table)
        {
            return ((NetworkTable)table).ToString().Replace("NetworkTable: ", "");
        }

        private string GetNodeFromTree(string tableName, string key)
        {
            string test = tree.Nodes[0].Nodes[0].FullPath;
            List<TreeNode> results = tree.Nodes.Cast<TreeNode>().Where(node => node.FullPath == tableName + key) as List<TreeNode>;

            /// Notes
            /// full path may be "SomeMech\\intakeMotorGains\\pGain (1.0)"
            /// network table name would be /SomeMech/intakeMotorGains plus the key which would be pGain
            /// so first, split table name by "/"
            /// next, find node with fullpath containg SomeMech .Where(node => node.FullPath.Contains(tableNameSplitArr[0]<SomeMech>)
            /// then recurse into that node and do the same thing, except now with intakeMotorGains
            /// keep iterating through table name until there's nothing left, then just find the node with the path containing the key

            return "";
        }

        private void UpdateNodeInTree(ITable table, string key, Value value)
        {

        }

        private void AddToTree(ITable table, string key, Value value)
        {
            throw new NotImplementedException();
        }

        public void EditNetworkTable(TreeNode node, string value)
        {
            string fullname = node.FullPath;

            string[] chunks = fullname.Split('/');

            foreach (string chunk in chunks)
            {

            }
        }

        public void AddListeners(NetworkTable table)
        {
            table.AddTableListenerEx(onTableChange, NotifyFlags.NotifyNew | NotifyFlags.NotifyUpdate | NotifyFlags.NotifyDelete);
            table.AddSubTableListener(onSubTableCreation);
            foreach (string subtable in table.GetSubTables())
            {
                NetworkTable subNT = (NetworkTable)table.GetSubTable(subtable);
                AddListeners(subNT);

                TreeNode newNode = new TreeNode(subtable);
                foreach (string key in subNT.GetKeys())
                {
                    newNode.Nodes.Add(key + "(" + GetNTValueAsString(subNT, key) + ")");
                }

                treeNodes.Add(newNode);

                tree.BeginInvoke(new Action(() =>
                {
                    tree.Nodes.Add(newNode);
                }));
            }
        }

        private string GetNTValueAsString(NetworkTable subNT, string key)
        {
            foreach (string testKey in subNT.GetKeys())
            {
                if (testKey == key)
                {
                    switch (subNT.GetValue(key, Value.MakeString("NO VALUE")).Type)
                    {
                        case NtType.Boolean:
                            return subNT.GetBoolean(key, false).ToString();
                            break;
                        case NtType.String:
                            return subNT.GetString(key, "NO VALUE");
                            break;
                        case NtType.Double:
                            return subNT.GetNumber(key, -1.302).ToString();
                            break;
                        default:
                            return "UNKNOWN TYPE";
                            break;
                    }
                }
            }
            return "UNKNOWN";
        }

        public void FilterTree(string text)
        {
            foreach (TreeNode node in treeNodes)
            {
                if (node != null)
                {
                    if (!node.Text.ToLower().Contains(text.ToLower()))
                    {
                        tree.Nodes.Remove(node);
                    }
                    else
                    {
                        if (!tree.Nodes.Contains(node))
                            tree.Nodes.Add(node);
                    }
                }
            }
        }

        public bool HasConnected()
        {
            return hasConnected;
        }
    }
}
// See https://aka.ms/new-console-template for more information
using KinectServer;
using System.IO;
using System.Text;
using System.Windows.Markup;


public class OfflinePostSync 
{
    static void Main(string[] args)
    {
        string pathToCapture = "C:\\OP_Entnahme_Verschluss_0\\";
        string[] clientFolders = Directory.GetDirectories(pathToCapture, "client_*");
        List<ClientData> clientData = new List<ClientData>();

        byte[] emptyColorBuffer = File.ReadAllBytes("emptyColor.jpg");
        byte[] emptyDepthBuffer = File.ReadAllBytes("emptyDepth.tiff");

        for (int i = 0; i < clientFolders.Length; i++)
        {
            clientData.Add(new ClientData(clientFolders[i]));
        }

        foreach (ClientData client in clientData)
        {
            ReadTimestampList(client);
        }

        List<ClientSyncData> allClientSyncData = new List<ClientSyncData>();
        foreach (ClientData client in clientData)
        {
            allClientSyncData.Add(client.syncData);
        }

        List<ClientSyncData> syncedData = PostSync.GenerateSyncList(allClientSyncData, pathToCapture);


        if(syncedData == null)
        {
            Console.WriteLine("Error: Could not successfully sync files!");
            return;
        }

        for (int i = 0; i < syncedData.Count; i++)
        {
            clientData[i].postsyncedData = syncedData[i];
        }

        for (int i = 0; i < clientData.Count; i++)
        {
            Console.WriteLine("Reordering files for client: " + i);
            ReorderFiles(clientData[i], emptyColorBuffer, emptyDepthBuffer);
        }

        Console.WriteLine("Postsync completed!");
    }

    static void ReadTimestampList(ClientData client)
    {
        string[] timestampList = Directory.GetFiles(client.clientDataPath, "Timestamps*");

        if (timestampList.Length > 1 || timestampList.Length == 0)
        {
            Console.WriteLine("ERROR: Could not find Timestamplist");
            return;
        }

        using (var fileStream = File.OpenRead(timestampList[0]))
        using (var streamReader = new StreamReader(fileStream, Encoding.UTF8, true, 512))
        {
            String line;
            while ((line = streamReader.ReadLine()) != null)
            {
                string[] values = line.Split('\t');
                client.frameNumbers.Add(int.Parse(values[0]));
                client.timestamps.Add(ulong.Parse(values[1]));
            }
        }

        string[] pathSplitted = client.clientDataPath.Split('_');
        int index = int.Parse(pathSplitted[pathSplitted.Length - 1]);
        client.syncData = new ClientSyncData(client.frameNumbers, client.timestamps, index);
    }

    static void ReorderFiles(ClientData client, byte[] emptyColorBuffer, byte[] emptyDepthBuffer)
    {
        for (int i = 0; i < client.postsyncedData.frames.Count; i++)
        {
            SyncFrame frame = client.postsyncedData.frames[i];

            string colorFileUnsyncedPath = client.clientDataPath + "\\Color_" + frame.frameID + ".jpg";
            string colorFileSyncedPath = client.clientDataPath + "\\synced_color_" + frame.syncedFrameID + ".jpg";

            string depthFileUnsyncedPath = client.clientDataPath + "\\Depth_" + frame.frameID + ".tiff";
            string depthFileSyncedPath = client.clientDataPath + "\\synced_depth_" + frame.syncedFrameID + ".tiff";

            if (frame.frameID == -1)
            {
                //No Data for this frame available, we need to substitute an empty picture
                File.WriteAllBytes(colorFileSyncedPath, emptyColorBuffer);
                File.WriteAllBytes(depthFileSyncedPath, emptyDepthBuffer);
                continue;
            }

            if (!File.Exists(colorFileUnsyncedPath))
            {
                if (!File.Exists(colorFileSyncedPath))
                    Console.WriteLine("Error, could not find file: " + colorFileUnsyncedPath);
            }

            else
                File.Move(colorFileUnsyncedPath, colorFileSyncedPath); //Rename pair



            if (!File.Exists(depthFileUnsyncedPath))
            {
                if (!File.Exists(depthFileSyncedPath))
                    Console.WriteLine("Error, could not find file: " + depthFileUnsyncedPath);
            }

            else
            {
                File.Move(depthFileUnsyncedPath, depthFileSyncedPath); //Rename pair
            }




        }
    }

}





class ClientData
{
    public string clientDataPath = "";
    public List<ulong> timestamps;
    public List<int> frameNumbers;
    public ClientSyncData syncData;
    public ClientSyncData postsyncedData;

    public ClientData(string pathToData)
    {
        clientDataPath = pathToData;
        timestamps = new List<ulong>();
        frameNumbers = new List<int>();
        syncData = new ClientSyncData();
    }
}
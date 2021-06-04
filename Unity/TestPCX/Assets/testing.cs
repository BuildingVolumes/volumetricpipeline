
using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using UnityEditor.Experimental.AssetImporters;

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Pcx;


public class MyReader
{
    const byte CarriageReturn = 0xD;
    const byte LineFeed = 0xA;
    byte[] fileBytes; // the entire file
    int Position; // stream head position/index into bytestream
    int lengthOfFile;
    char[] temp;

    public MyReader(){
        Position = 0;
        fileBytes = null;
        temp = new char[255];
    }
    
    public void ReadFile(string fname){
        if(fileBytes != null) fileBytes = null;

        fileBytes = File.ReadAllBytes(fname);
        lengthOfFile = fileBytes.Length;
        Position = 0;
    }

    public string ReadLine(){
        int start = Position; // start pos
        int linebreakPos = GetLineBreak(Position, fileBytes);
       // Debug.Log("lineBreak="+linebreakPos);
        int numBytes = linebreakPos - start; // length from start to line break 
        //Debug.Log("numBytes = "+numBytes);
        string s = "";
        if(linebreakPos > -1){
            for(int i=start;i<start+numBytes;i++)
            {
                s += (char)fileBytes[i];
                //Debug.Log((char) (fileBytes[i]) + " :: " + (byte) fileBytes[i] + " :: " + (int) fileBytes[i]);
            }
            AdvanceFilePositionByNumBytes(numBytes+1); // advance Position 
        }
        return s;
    }

    public void SetFilePosition(int p){
        Position = p;
    }
    public void AdvanceFilePositionByNumBytes(int num){
        Position += num;
    }

    public byte ReadByte(){
        byte r = fileBytes[Position];
        AdvanceFilePositionByNumBytes(1);
        return r;
    }
    public ushort ReadUInt16(){
        // get 4 bytes and put it into an int
        ushort r = BitConverter.ToUInt16(fileBytes, Position);
        AdvanceFilePositionByNumBytes(2);
        return r;
    }
    public float ReadSingle(){
        float r = BitConverter.ToSingle(fileBytes, Position);
        AdvanceFilePositionByNumBytes(4);
        return r;
    }
    public double ReadDouble(){
        double r = BitConverter.ToDouble(fileBytes, Position);
        AdvanceFilePositionByNumBytes(8);
        return r;
    }
    

    public int GetLineBreak(int startIndex, byte[] bytes)
    {
        int len = lengthOfFile;//unicodeBytes.Length;
        for (int pos = startIndex; pos < len - 1; pos++)
        {
            if (bytes[pos] == (byte)'\n') // '\n'
                return pos;
        }
        return -1;
    }
    
    // public static IEnumerable<string> GetAllLines(string file)
    // {
    //     byte[] unicodeBytes = File.ReadAllBytes(file);
    //     int pos = 0, lastPos = 0;
    //     while ((pos = GetLineBreak(pos, unicodeBytes)) > -1)
    //     {
    //         yield return Encoding.Unicode.GetString(unicodeBytes, lastPos, pos - lastPos);
    //         lastPos = pos;
    //         pos += 4;
    //     }
    //     if (lastPos < unicodeBytes.Length - 2)
    //     {
    //         yield return Encoding.Unicode.GetString(unicodeBytes, lastPos, unicodeBytes.Length - lastPos);
    //     }
    // }
}


public class testing : MonoBehaviour
{
    public string PlySequenceDirectory;
    public string Prefix="";
    public int NumDigits=5;
    public int NumFilesInSequence=1;
    public int StartFromNumber=0;
    MyReader theReader;
 
    int count=0;
    int FC,FCStart;
    GameObject gameObject;
    // Start is called before the first frame update
    void Start()
    {
     	gameObject = new GameObject();
        string filename = GetFileName();
        theReader = new MyReader();

 		var mesh = ImportAsMesh(filename);
        var meshFilter = gameObject.AddComponent<MeshFilter>();
        meshFilter.sharedMesh = mesh;
        var meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = GetDefaultMaterial();
        count = StartFromNumber;
        FCStart = Time.frameCount;
    }

    string GetFileName()
    {
        string D = "d"+NumDigits.ToString();
        count = (FC - FCStart) + StartFromNumber;
        return PlySequenceDirectory +"/"+ Prefix + count.ToString(D) + ".ply";
    }
    // Update is called once per frame
    void Update()
    {
        
        FC = Time.frameCount;

        string filename = GetFileName();

        MeshFilter meshRenderer = gameObject.GetComponent(typeof(MeshFilter)) as MeshFilter;
        
        Mesh mesh = ImportAsMesh(filename);
        meshRenderer.sharedMesh = mesh;
        count++;
        if(count > NumFilesInSequence){
            count = 0;
            FCStart = FC;
        }
    }
        Mesh ImportAsMesh(string filename)
        {
            try
            {
                theReader.ReadFile(filename);
                var header = ReadDataHeader();
                var body = ReadDataBody(header);

                var mesh = new Mesh();
                mesh.name = Path.GetFileNameWithoutExtension(filename);

                mesh.indexFormat = header.vertexCount > 65535 ?
                    IndexFormat.UInt32 : IndexFormat.UInt16;

                mesh.SetVertices(body.vertices);
                mesh.SetColors(body.colors);

                mesh.SetIndices(
                    Enumerable.Range(0, header.vertexCount).ToArray(),
                    MeshTopology.Points, 0
                );

                mesh.UploadMeshData(true);
                return mesh;
            }
            catch (Exception e)
            {
                Debug.LogError("Failed importing " + filename + ". " + e.Message);
                return null;
            }
        }
        // Mesh ImportAsMesh(string path)
        // {
        //     try
        //     {
        //         var stream = File.Open(path, FileMode.Open, FileAccess.Read, FileShare.Read);
        //         var header = ReadDataHeader(new StreamReader(stream));
        //         var body = ReadDataBody(header, new BinaryReader(stream));

        //         var mesh = new Mesh();
        //         mesh.name = Path.GetFileNameWithoutExtension(path);

        //         mesh.indexFormat = header.vertexCount > 65535 ?
        //             IndexFormat.UInt32 : IndexFormat.UInt16;

        //         mesh.SetVertices(body.vertices);
        //         mesh.SetColors(body.colors);

        //         mesh.SetIndices(
        //             Enumerable.Range(0, header.vertexCount).ToArray(),
        //             MeshTopology.Points, 0
        //         );

        //         mesh.UploadMeshData(true);
        //         return mesh;
        //     }
        //     catch (Exception e)
        //     {
        //         Debug.LogError("Failed importing " + path + ". " + e.Message);
        //         return null;
        //     }
        // }
     // PointCloudData ImportAsPointCloudData(string path)
     //    {
     //        try
     //        {
     //            var stream = File.Open(path, FileMode.Open, FileAccess.Read, FileShare.Read);
     //            var header = ReadDataHeader(new StreamReader(stream));
     //            var body = ReadDataBody(header, new BinaryReader(stream));
     //            var data = ScriptableObject.CreateInstance<PointCloudData>();
     //            data.Initialize(body.vertices, body.colors);
     //            data.name = Path.GetFileNameWithoutExtension(path);
     //            return data;
     //        }
     //        catch (Exception e)
     //        {
     //            Debug.LogError("Failed importing " + path + ". " + e.Message);
     //            return null;
     //        }
     //    }

        static Material GetDefaultMaterial()
        {
            // Via package manager
            var path_upm = "Packages/jp.keijiro.pcx/Editor/Default Point.mat";
            // Via project asset database
            var path_prj = "Assets/Pcx/Editor/Default Point.mat";
            return AssetDatabase.LoadAssetAtPath<Material>(path_upm) ??
                   AssetDatabase.LoadAssetAtPath<Material>(path_prj);
        }
 enum DataProperty {
            Invalid,
            R8, G8, B8, A8,
            R16, G16, B16, A16,
            SingleX, SingleY, SingleZ,
            DoubleX, DoubleY, DoubleZ,
            Data8, Data16, Data32, Data64
        }

        static int GetPropertySize(DataProperty p)
        {
            switch (p)
            {
                case DataProperty.R8: return 1;
                case DataProperty.G8: return 1;
                case DataProperty.B8: return 1;
                case DataProperty.A8: return 1;
                case DataProperty.R16: return 2;
                case DataProperty.G16: return 2;
                case DataProperty.B16: return 2;
                case DataProperty.A16: return 2;
                case DataProperty.SingleX: return 4;
                case DataProperty.SingleY: return 4;
                case DataProperty.SingleZ: return 4;
                case DataProperty.DoubleX: return 8;
                case DataProperty.DoubleY: return 8;
                case DataProperty.DoubleZ: return 8;
                case DataProperty.Data8: return 1;
                case DataProperty.Data16: return 2;
                case DataProperty.Data32: return 4;
                case DataProperty.Data64: return 8;
            }
            return 0;
        }

        class DataHeader
        {
            public List<DataProperty> properties = new List<DataProperty>();
            public int vertexCount = -1;
        }

        class DataBody
        {
            public List<Vector3> vertices;
            public List<Color32> colors;

            public DataBody(int vertexCount)
            {
                vertices = new List<Vector3>(vertexCount);
                colors = new List<Color32>(vertexCount);
            }

            public void AddPoint(
                float x, float y, float z,
                byte r, byte g, byte b, byte a
            )
            {
                vertices.Add(new Vector3(x, y, z));
                colors.Add(new Color32(r, g, b, a));
            }
        }


        DataHeader ReadDataHeader()
        {
            var data = new DataHeader();
            var readCount = 0;

            // Magic number line ("ply")
            var line = theReader.ReadLine();
            //Debug.Log("line="+line);
            readCount += line.Length + 1;
            if (line != "ply")
                throw new ArgumentException("Magic number ('ply') mismatch.");

            // Data format: check if it's binary/little endian.
            line = theReader.ReadLine();
            //Debug.Log("line="+line);
            readCount += line.Length + 1;
            if (line != "format binary_little_endian 1.0")
                throw new ArgumentException(
                    "Invalid data format ('" + line + "'). " +
                    "Should be binary_little_endian.");
            // Read header contents.
            for (var skip = false;;)
            {
                // Read a line and split it with white space.
                line = theReader.ReadLine();
                //Debug.Log("line="+line);
                readCount += line.Length + 1;
                if (line == "end_header") break;
                var col = line.Split();

                // Element declaration (unskippable)
                if (col[0] == "element")
                {
                    //Debug.Log("ELEMENT PARSING");
                    if (col[1] == "vertex")
                    {
                        data.vertexCount = Convert.ToInt32(col[2]);
                       // Debug.Log("VERTEX COUNT="+data.vertexCount);
                        skip = false;
                    }
                    else
                    {
                        // Don't read elements other than vertices.
                        skip = true;
                    }
                }

                //count++;
                //if(count > 100) return null;
                if (skip) continue;

                // Property declaration line
                if (col[0] == "property")
                {
                    //Debug.Log("PROPERY LIST PARSING");
                    var prop = DataProperty.Invalid;

                    // Parse the property name entry.
                    switch (col[2])
                    {
                        case "red"  : prop = DataProperty.R8; break;
                        case "green": prop = DataProperty.G8; break;
                        case "blue" : prop = DataProperty.B8; break;
                        case "alpha": prop = DataProperty.A8; break;
                        case "x"    : prop = DataProperty.SingleX; break;
                        case "y"    : prop = DataProperty.SingleY; break;
                        case "z"    : prop = DataProperty.SingleZ; break;
                    }

                    // Check the property type.
                    if (col[1] == "char" || col[1] == "uchar" ||
                        col[1] == "int8" || col[1] == "uint8")
                    {
                        if (prop == DataProperty.Invalid)
                            prop = DataProperty.Data8;
                        else if (GetPropertySize(prop) != 1)
                            throw new ArgumentException("Invalid property type ('" + line + "').");
                    }
                    else if (col[1] == "short" || col[1] == "ushort" ||
                             col[1] == "int16" || col[1] == "uint16")
                    {
                        switch (prop)
                        {
                            case DataProperty.Invalid: prop = DataProperty.Data16; break;
                            case DataProperty.R8: prop = DataProperty.R16; break;
                            case DataProperty.G8: prop = DataProperty.G16; break;
                            case DataProperty.B8: prop = DataProperty.B16; break;
                            case DataProperty.A8: prop = DataProperty.A16; break;
                        }
                        if (GetPropertySize(prop) != 2)
                            throw new ArgumentException("Invalid property type ('" + line + "').");
                    }
                    else if (col[1] == "int"   || col[1] == "uint"   || col[1] == "float" ||
                             col[1] == "int32" || col[1] == "uint32" || col[1] == "float32")
                    {
                        if (prop == DataProperty.Invalid)
                            prop = DataProperty.Data32;
                        else if (GetPropertySize(prop) != 4)
                            throw new ArgumentException("Invalid property type ('" + line + "').");
                    }
                    else if (col[1] == "int64"  || col[1] == "uint64" ||
                             col[1] == "double" || col[1] == "float64")
                    {
                        switch (prop)
                        {
                            case DataProperty.Invalid: prop = DataProperty.Data64; break;
                            case DataProperty.SingleX: prop = DataProperty.DoubleX; break;
                            case DataProperty.SingleY: prop = DataProperty.DoubleY; break;
                            case DataProperty.SingleZ: prop = DataProperty.DoubleZ; break;
                        }
                        if (GetPropertySize(prop) != 8)
                            throw new ArgumentException("Invalid property type ('" + line + "').");
                    }
                    else
                    {
                        throw new ArgumentException("Unsupported property type ('" + line + "').");
                    }

                    data.properties.Add(prop);
                }
            }

            // Rewind the stream back to the exact position of the reader.
            theReader.SetFilePosition(readCount);

            return data;
        }

        // DataHeader ReadDataHeader(StreamReader reader)
        // {
        //     var data = new DataHeader();
        //     var readCount = 0;

        //     // Magic number line ("ply")
        //     var line = reader.ReadLine();
        //     readCount += line.Length + 1;
        //     if (line != "ply")
        //         throw new ArgumentException("Magic number ('ply') mismatch.");

        //     // Data format: check if it's binary/little endian.
        //     line = reader.ReadLine();
        //     readCount += line.Length + 1;
        //     if (line != "format binary_little_endian 1.0")
        //         throw new ArgumentException(
        //             "Invalid data format ('" + line + "'). " +
        //             "Should be binary/little endian.");

        //     // Read header contents.
        //     for (var skip = false;;)
        //     {
        //         // Read a line and split it with white space.
        //         line = reader.ReadLine();
        //         readCount += line.Length + 1;
        //         if (line == "end_header") break;
        //         var col = line.Split();

        //         // Element declaration (unskippable)
        //         if (col[0] == "element")
        //         {
        //             if (col[1] == "vertex")
        //             {
        //                 data.vertexCount = Convert.ToInt32(col[2]);
        //                 skip = false;
        //             }
        //             else
        //             {
        //                 // Don't read elements other than vertices.
        //                 skip = true;
        //             }
        //         }

        //         if (skip) continue;

        //         // Property declaration line
        //         if (col[0] == "property")
        //         {
        //             var prop = DataProperty.Invalid;

        //             // Parse the property name entry.
        //             switch (col[2])
        //             {
        //                 case "red"  : prop = DataProperty.R8; break;
        //                 case "green": prop = DataProperty.G8; break;
        //                 case "blue" : prop = DataProperty.B8; break;
        //                 case "alpha": prop = DataProperty.A8; break;
        //                 case "x"    : prop = DataProperty.SingleX; break;
        //                 case "y"    : prop = DataProperty.SingleY; break;
        //                 case "z"    : prop = DataProperty.SingleZ; break;
        //             }

        //             // Check the property type.
        //             if (col[1] == "char" || col[1] == "uchar" ||
        //                 col[1] == "int8" || col[1] == "uint8")
        //             {
        //                 if (prop == DataProperty.Invalid)
        //                     prop = DataProperty.Data8;
        //                 else if (GetPropertySize(prop) != 1)
        //                     throw new ArgumentException("Invalid property type ('" + line + "').");
        //             }
        //             else if (col[1] == "short" || col[1] == "ushort" ||
        //                      col[1] == "int16" || col[1] == "uint16")
        //             {
        //                 switch (prop)
        //                 {
        //                     case DataProperty.Invalid: prop = DataProperty.Data16; break;
        //                     case DataProperty.R8: prop = DataProperty.R16; break;
        //                     case DataProperty.G8: prop = DataProperty.G16; break;
        //                     case DataProperty.B8: prop = DataProperty.B16; break;
        //                     case DataProperty.A8: prop = DataProperty.A16; break;
        //                 }
        //                 if (GetPropertySize(prop) != 2)
        //                     throw new ArgumentException("Invalid property type ('" + line + "').");
        //             }
        //             else if (col[1] == "int"   || col[1] == "uint"   || col[1] == "float" ||
        //                      col[1] == "int32" || col[1] == "uint32" || col[1] == "float32")
        //             {
        //                 if (prop == DataProperty.Invalid)
        //                     prop = DataProperty.Data32;
        //                 else if (GetPropertySize(prop) != 4)
        //                     throw new ArgumentException("Invalid property type ('" + line + "').");
        //             }
        //             else if (col[1] == "int64"  || col[1] == "uint64" ||
        //                      col[1] == "double" || col[1] == "float64")
        //             {
        //                 switch (prop)
        //                 {
        //                     case DataProperty.Invalid: prop = DataProperty.Data64; break;
        //                     case DataProperty.SingleX: prop = DataProperty.DoubleX; break;
        //                     case DataProperty.SingleY: prop = DataProperty.DoubleY; break;
        //                     case DataProperty.SingleZ: prop = DataProperty.DoubleZ; break;
        //                 }
        //                 if (GetPropertySize(prop) != 8)
        //                     throw new ArgumentException("Invalid property type ('" + line + "').");
        //             }
        //             else
        //             {
        //                 throw new ArgumentException("Unsupported property type ('" + line + "').");
        //             }

        //             data.properties.Add(prop);
        //         }
        //     }

        //     // Rewind the stream back to the exact position of the reader.
        //     reader.BaseStream.Position = readCount;

        //     return data;
        // }
        DataBody ReadDataBody(DataHeader header)
        {
           // Debug.Log(" READ DATA BODY!!");

            var data = new DataBody(header.vertexCount);

            float x = 0, y = 0, z = 0;
            Byte r = 255, g = 255, b = 255, a = 255;

            for (var i = 0; i < header.vertexCount; i++)
            {
                foreach (var prop in header.properties)
                {
                    switch (prop)
                    {
                        case DataProperty.R8: r = theReader.ReadByte(); break;
                        case DataProperty.G8: g = theReader.ReadByte(); break;
                        case DataProperty.B8: b = theReader.ReadByte(); break;
                        case DataProperty.A8: a = theReader.ReadByte(); break;

                        case DataProperty.R16: r = (byte)(theReader.ReadUInt16() >> 8); break;
                        case DataProperty.G16: g = (byte)(theReader.ReadUInt16() >> 8); break;
                        case DataProperty.B16: b = (byte)(theReader.ReadUInt16() >> 8); break;
                        case DataProperty.A16: a = (byte)(theReader.ReadUInt16() >> 8); break;

                        case DataProperty.SingleX: x = theReader.ReadSingle(); break;
                        case DataProperty.SingleY: y = theReader.ReadSingle(); break;
                        case DataProperty.SingleZ: z = theReader.ReadSingle(); break;

                        case DataProperty.DoubleX: x = (float)theReader.ReadDouble(); break;
                        case DataProperty.DoubleY: y = (float)theReader.ReadDouble(); break;
                        case DataProperty.DoubleZ: z = (float)theReader.ReadDouble(); break;

                        case DataProperty.Data8: theReader.ReadByte(); break;
                        case DataProperty.Data16: theReader.AdvanceFilePositionByNumBytes(2); break;
                        case DataProperty.Data32: theReader.AdvanceFilePositionByNumBytes(4); break;
                        case DataProperty.Data64: theReader.AdvanceFilePositionByNumBytes(8); break;
                    }
                }

                data.AddPoint(x, y, z, r, g, b, a);
                //Debug.Log("(x,y,z)="+x+","+y+","+z+")--"+"(r,g,b,a)="+r+","+g+","+b+","+a+")");
            }

            return data;
        }
        // DataBody ReadDataBody(DataHeader header, BinaryReader reader)
        // {
        //     var data = new DataBody(header.vertexCount);

        //     float x = 0, y = 0, z = 0;
        //     Byte r = 255, g = 255, b = 255, a = 255;

        //     for (var i = 0; i < header.vertexCount; i++)
        //     {
        //         foreach (var prop in header.properties)
        //         {
        //             switch (prop)
        //             {
        //                 case DataProperty.R8: r = reader.ReadByte(); break;
        //                 case DataProperty.G8: g = reader.ReadByte(); break;
        //                 case DataProperty.B8: b = reader.ReadByte(); break;
        //                 case DataProperty.A8: a = reader.ReadByte(); break;

        //                 case DataProperty.R16: r = (byte)(reader.ReadUInt16() >> 8); break;
        //                 case DataProperty.G16: g = (byte)(reader.ReadUInt16() >> 8); break;
        //                 case DataProperty.B16: b = (byte)(reader.ReadUInt16() >> 8); break;
        //                 case DataProperty.A16: a = (byte)(reader.ReadUInt16() >> 8); break;

        //                 case DataProperty.SingleX: x = reader.ReadSingle(); break;
        //                 case DataProperty.SingleY: y = reader.ReadSingle(); break;
        //                 case DataProperty.SingleZ: z = reader.ReadSingle(); break;

        //                 case DataProperty.DoubleX: x = (float)reader.ReadDouble(); break;
        //                 case DataProperty.DoubleY: y = (float)reader.ReadDouble(); break;
        //                 case DataProperty.DoubleZ: z = (float)reader.ReadDouble(); break;

        //                 case DataProperty.Data8: reader.ReadByte(); break;
        //                 case DataProperty.Data16: reader.BaseStream.Position += 2; break;
        //                 case DataProperty.Data32: reader.BaseStream.Position += 4; break;
        //                 case DataProperty.Data64: reader.BaseStream.Position += 8; break;
        //             }
        //         }

        //         data.AddPoint(x, y, z, r, g, b, a);
        //     }

        //     return data;
        // }



}

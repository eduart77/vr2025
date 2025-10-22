using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class CtScan: Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public CtScan(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }
    
    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        // Check if ray intersects with the bounding box of the volume
        double tMin = double.MaxValue;
        double tMax = double.MinValue;
        
        // Calculate intersection with volume bounding box
        for (int i = 0; i < 3; i++)
        {
            double rayDir = i == 0 ? line.Dx.X : (i == 1 ? line.Dx.Y : line.Dx.Z);
            double rayOrigin = i == 0 ? line.X0.X : (i == 1 ? line.X0.Y : line.X0.Z);
            double boxMin = i == 0 ? _v0.X : (i == 1 ? _v0.Y : _v0.Z);
            double boxMax = i == 0 ? _v1.X : (i == 1 ? _v1.Y : _v1.Z);
            
            if (Math.Abs(rayDir) < 1e-8)
            {
                // Ray is parallel to the plane
                if (rayOrigin < boxMin || rayOrigin > boxMax)
                {
                    return Intersection.NONE;
                }
            }
            else
            {
                double t1 = (boxMin - rayOrigin) / rayDir;
                double t2 = (boxMax - rayOrigin) / rayDir;
                
                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }
                
                tMin = Math.Max(tMin, t1);
                tMax = Math.Min(tMax, t2);
                
                if (tMin > tMax)
                {
                    return Intersection.NONE;
                }
            }
        }
        
        // Clamp to the requested distance range
        tMin = Math.Max(tMin, minDist);
        tMax = Math.Min(tMax, maxDist);
        
        if (tMin > tMax)
        {
            return Intersection.NONE;
        }
        
        // Ray-march through the volume
        double stepSize = Math.Min(_thickness[0], Math.Min(_thickness[1], _thickness[2])) * _scale * 0.5;
        double t = tMin;
        
        while (t <= tMax)
        {
            Vector position = line.CoordinateToPosition(t);
            
            // Check if position is within volume bounds
            if (position.X >= _v0.X && position.X <= _v1.X &&
                position.Y >= _v0.Y && position.Y <= _v1.Y &&
                position.Z >= _v0.Z && position.Z <= _v1.Z)
            {
                int[] idx = GetIndexes(position);
                ushort value = Value(idx[0], idx[1], idx[2]);
                
                // Check if we hit a non-zero density (assuming 0 is empty space)
                if (value > 0)
                {
                    Color color = GetColor(position);
                    Vector normal = GetNormal(position);
                    
                    return new Intersection(true, true, this, line, t, normal, Material.FromColor(color), color);
                }
            }
            
            t += stepSize;
        }
        
        return Intersection.NONE;
    }
    
    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
    }
    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);

        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(Vector v)
    {
        int[] idx = GetIndexes(v);
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}
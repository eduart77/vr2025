using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            // Transform ray to ellipsoid's local coordinate system
            Vector rayOrigin = line.X0 - Center;
            Vector rayDirection = line.Dx;
            
            // Scale the ray origin and direction by the inverse semi-axes lengths
            Vector scaledOrigin = new Vector(
                rayOrigin.X / SemiAxesLength.X,
                rayOrigin.Y / SemiAxesLength.Y,
                rayOrigin.Z / SemiAxesLength.Z
            );
            
            Vector scaledDirection = new Vector(
                rayDirection.X / SemiAxesLength.X,
                rayDirection.Y / SemiAxesLength.Y,
                rayDirection.Z / SemiAxesLength.Z
            );
            
            // Solve quadratic equation for ellipsoid intersection
            // (scaledOrigin + t * scaledDirection) · (scaledOrigin + t * scaledDirection) = radius²
            double a = scaledDirection * scaledDirection;
            double b = 2.0 * (scaledOrigin * scaledDirection);
            double c = scaledOrigin * scaledOrigin - Radius * Radius;
            
            double discriminant = b * b - 4.0 * a * c;
            
            if (discriminant < 0)
            {
                return Intersection.NONE;
            }
            
            double sqrtDiscriminant = Math.Sqrt(discriminant);
            double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
            double t2 = (-b + sqrtDiscriminant) / (2.0 * a);
            
            // Find the closest valid intersection
            double t = double.MaxValue;
            if (t1 >= minDist && t1 <= maxDist)
            {
                t = t1;
            }
            else if (t2 >= minDist && t2 <= maxDist)
            {
                t = t2;
            }
            else
            {
                return Intersection.NONE;
            }
            
            Vector intersectionPoint = line.CoordinateToPosition(t);
            Vector normal = GetNormal(intersectionPoint);
            
            return new Intersection(true, true, this, line, t, normal, Material, Color);
        }
        
        private Vector GetNormal(Vector point)
        {
            Vector localPoint = point - Center;
            Vector normal = new Vector(
                2.0 * localPoint.X / (SemiAxesLength.X * SemiAxesLength.X),
                2.0 * localPoint.Y / (SemiAxesLength.Y * SemiAxesLength.Y),
                2.0 * localPoint.Z / (SemiAxesLength.Z * SemiAxesLength.Z)
            );
            return normal.Normalize();
        }
    }
}

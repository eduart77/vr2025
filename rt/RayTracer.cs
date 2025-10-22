using System;

namespace rt
{
    class RayTracer(Geometry[] geometries, Light[] lights)
    {
        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {
            Vector lightDirection = light.Position - point;
            double lightDistance = lightDirection.Length();
            lightDirection.Normalize();
            
            Line shadowRay = new Line(point, light.Position);
            
            // Check for intersections with geometries (excluding the current point)
            foreach (var geometry in geometries)
            {
                var intersection = geometry.GetIntersection(shadowRay, 0.001, lightDistance - 0.001);
                if (intersection.Valid && intersection.Visible)
                {
                    return false; // Point is in shadow
                }
            }
            
            return true; // Point is lit
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);

            var image = new Image(width, height);

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    // Convert pixel coordinates to view plane coordinates
                    double x = ImageToViewPlane(i, width, camera.ViewPlaneWidth);
                    double y = ImageToViewPlane(j, height, camera.ViewPlaneHeight);
                    
                    // Create ray from camera through pixel
                    Vector right = camera.Direction ^ camera.Up;
                    right.Normalize();
                    Vector pixelPosition = camera.Position + camera.Direction * camera.ViewPlaneDistance + 
                                         right * x + camera.Up * y;
                    Line ray = new Line(camera.Position, pixelPosition);
                    
                    // Find first intersection
                    var intersection = FindFirstIntersection(ray, 0.001, double.MaxValue);
                    
                    Color pixelColor;
                    if (intersection.Valid && intersection.Visible)
                    {
                        // Calculate lighting for the intersection point
                        pixelColor = CalculateLighting(intersection);
                    }
                    else
                    {
                        pixelColor = background;
                    }
                    
                    image.SetPixel(i, j, pixelColor);
                }
            }

            image.Store(filename);
        }
        
        private Color CalculateLighting(Intersection intersection)
        {
            Color ambient = new Color();
            Color diffuse = new Color();
            Color specular = new Color();
            
            foreach (var light in lights)
            {
                // Ambient lighting
                ambient += intersection.Material.Ambient * light.Ambient;
                
                if (IsLit(intersection.Position, light))
                {
                    Vector lightDirection = light.Position - intersection.Position;
                    lightDirection.Normalize();
                    
                    // Diffuse lighting
                    double diffuseFactor = Math.Max(0, intersection.Normal * lightDirection);
                    diffuse += intersection.Material.Diffuse * light.Diffuse * diffuseFactor;
                    
                    // Specular lighting
                    Vector viewDirection = (new Vector(0, 0, 0) - intersection.Position).Normalize();
                    Vector reflectDirection = intersection.Normal * (2.0 * (intersection.Normal * lightDirection)) - lightDirection;
                    double specularFactor = Math.Pow(Math.Max(0, viewDirection * reflectDirection), intersection.Material.Shininess);
                    specular += intersection.Material.Specular * light.Specular * specularFactor;
                }
            }
            
            // Combine lighting components with object color
            Color finalColor = (ambient + diffuse + specular) * intersection.Color;
            
            return finalColor;
        }
    }
}
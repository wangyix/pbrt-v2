// cameras/realistic.cpp*
#include "stdafx.h"
#include "cameras/realistic.h"
#include "paramset.h"
#include "sampler.h"
#include "montecarlo.h"
#include "filters/box.h"
#include "film/image.h"
#include "samplers/stratified.h"
#include "samplers/random.h"
#include "intersection.h"
#include "renderer.h"

#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

using namespace std;

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film) {
	   // Extract common camera parameters from \use{ParamSet}
	   float hither = params.FindOneFloat("hither", -1);
	   float yon = params.FindOneFloat("yon", -1);
	   float shutteropen = params.FindOneFloat("shutteropen", -1);
	   float shutterclose = params.FindOneFloat("shutterclose", -1);

	   // Realistic camera-specific parameters
	   string specfile = params.FindOneString("specfile", "");
	   float filmdistance = params.FindOneFloat("filmdistance", 70.0); // about 70 mm default to film
	   float fstop = params.FindOneFloat("aperture_diameter", 1.0);
	   float filmdiag = params.FindOneFloat("filmdiag", 35.0);
	   string autofocusfile = params.FindOneString("af_zones", "");
	   assert(hither != -1 && yon != -1 && shutteropen != -1 &&
	      shutterclose != -1 && filmdistance!= -1);
	   if (specfile == "") {
	       Severe( "No lens spec file supplied!\n" );
	   }
	   return new RealisticCamera(cam2world, hither, yon,
	      shutteropen, shutterclose, filmdistance, fstop,
	      specfile, autofocusfile, filmdiag, film);
}

RealisticCamera::RealisticCamera(const AnimatedTransform &cam2world,
                                 float hither, float yon,
                                 float sopen, float sclose,
                                 float filmdistance, float aperture_diameter_,
                                 const string &specfile,
								 const string &autofocusfile,
                                 float filmdiag,
								 Film *f)
                                 : Camera(cam2world, sopen, sclose, f),
								   ShutterOpen(sopen),
								   ShutterClose(sclose),
								   film(f),
                                   filmDistance(filmdistance * 0.001f),
                                   apertureDiameter(aperture_diameter_ * 0.001f)
{


	// YOUR CODE HERE -- build and store datastructures representing the given lens
	// and film placement.

    char line[512];

    // parse lens file
    ifstream ifs(specfile.c_str());
    if (!ifs) {
        fprintf(stderr, "Cannot open file %s\n", specfile.c_str());
        exit(-1);
    }

    float lensZIntercept = 0.f;
    float lastRefractiveIndex = 1.f;
        
    while (!ifs.eof()) {
        ifs.getline(line, 512);
        if (line[0] != '\0' && line[0] != '#' &&
            line[0] != ' ' && line[0] != '\t' && line[0] != '\n') {

            float sphereRadius, lensZThickness, refractiveIndex, aperture;
            sscanf(line, "%f %f %f %f\n", &sphereRadius, &lensZThickness,
                &refractiveIndex, &aperture);

            sphereRadius *= 0.001f;
            lensZThickness *= 0.001f;
            aperture *= 0.001f;

            if (sphereRadius == 0.f) {
                // aperture stop
                refractiveIndex = 1.f;
                if (apertureDiameter > aperture) {
                    Warning("Aperture diameter [%f] larger than max aperture [%f] in specfile.  Clamping to max.",
                        apertureDiameter, aperture);
                    apertureDiameter = aperture;
                }
            }
            

            LensSurface lensSurface;
            lensSurface.zIntercept = lensZIntercept;
            lensSurface.sphereRadius = abs(sphereRadius);
            lensSurface.sphereCenterZ = lensZIntercept - sphereRadius;
            lensSurface.refractiveRatio = refractiveIndex / lastRefractiveIndex;
            lensSurface.aperture = aperture;
            
            lensSurfaces.push_back(lensSurface);

            lensZIntercept -= lensZThickness;
            lastRefractiveIndex = refractiveIndex;
        }
    }
    ifs.close();

    // compute area of rear lens disk
    const LensSurface &rearLens = lensSurfaces.back();

    // compute raster to camera transform
    float filmZIntercept = rearLens.zIntercept - this->filmDistance;
    float rasterDiag = sqrtf(film->xResolution*film->xResolution + 
                             film->yResolution*film->yResolution);
    float s = filmdiag * 0.001f / rasterDiag;
    RasterToCamera = Scale(-s, s, 1.f) *
        Translate(Vector(-0.5f*film->xResolution, -0.5f*film->yResolution, filmZIntercept));

    // compute rear lens disk (disk's edge coincides with edge of rear lens)
    float diskRadius = 0.5f * rearLens.aperture;;
    float diskZ;
    if (rearLens.sphereRadius == 0.f) {
        diskZ = rearLens.zIntercept;
    } else {
        float r = rearLens.sphereRadius;
        float s = diskRadius;
        assert(r >= s);
        float diskZFromSphereCenter = sqrtf(r*r - s*s);
        diskZ = (rearLens.zIntercept < rearLens.sphereCenterZ) ?
              rearLens.sphereCenterZ - diskZFromSphereCenter
            : rearLens.sphereCenterZ + diskZFromSphereCenter;
    }
    RearLensDiskToCamera = Scale(diskRadius, diskRadius, 1.f)
                         * Translate(Vector(0.f, 0.f, diskZ));

    // compute area of rear lens disk
    rearLensDiskArea = M_PI * diskRadius * diskRadius;


	// If 'autofocusfile' is the empty string, then you should do
	// nothing in any subsequent call to AutoFocus()
	autofocus = false;

	if (autofocusfile.compare("") != 0)  {
		ParseAfZones(autofocusfile);
		autofocus = true;
	}
}


// parses the AF zone file
void RealisticCamera::ParseAfZones(const string& filename)
{
  ifstream specfile(filename.c_str());
   if (!specfile) {
      fprintf(stderr, "Cannot open file %s\n", filename.c_str());
      exit (-1);
   }

   char line[512];

   while (!specfile.eof()) {
      specfile.getline(line, 512);
      if (line[0] != '\0' && line[0] != '#' &&
         line[0] != ' ' && line[0] != '\t' && line[0] != '\n')
      {
		afZones.resize(afZones.size()+1);
		AfZone& zone = afZones[afZones.size()-1];
		sscanf(line, "%f %f %f %f\n", &zone.left, &zone.right, &zone.top, &zone.bottom);
      }
   }

	printf("Read in %zu AF zones from %s\n", afZones.size(), filename.c_str());
}

RealisticCamera::~RealisticCamera()
{

}

float RealisticCamera::GenerateRay(const CameraSample &sample, Ray *ray) const
{
  // YOUR CODE HERE -- make that ray!

  // use sample->imageX and sample->imageY to get raster-space coordinates
  // of the sample point on the film.
  // use sample->lensU and sample->lensV to get a sample position on the lens

  // GenerateRay() should return the weight of the generated ray

    // compute raster point in camera space
    Point Pras(sample.imageX, sample.imageY, 0);
    Point Pras_c;
    RasterToCamera(Pras, &Pras_c);

    // compute point on rear lens disk in camera space
    float lensU, lensV;
    ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
    Point Pdisk(lensU, lensV, 0);
    Point Pdisk_c;
    RearLensDiskToCamera(Pdisk, &Pdisk_c);

    *ray = Ray(Pras_c, Normalize(Pdisk_c - Pras_c), 0.f, INFINITY);
    float filmRayDirZ = ray->d.z;

    // intersect ray with lens surfaces from rear to front
    for (int i = lensSurfaces.size() - 1; i >= 0; i--) {

        const LensSurface &lensSurface = lensSurfaces[i];
        
        Point intersect;
        Vector normal;
        float R = lensSurface.sphereRadius;
        if (R == 0.f) {
            // lens surface is planar; intersect with lens plane
            if (ray->d.z == 0.f)
                return 0.f;
            float t = (lensSurface.sphereCenterZ - ray->o.z) / ray->d.z;
            intersect = (*ray)(t);
            normal = Vector(0.f, 0.f, -1.f);
            assert(t > 0.f);
        } else {
            // intersect with the sphere of the lens
            Point C = Point(0.f, 0.f, lensSurface.sphereCenterZ);
            Vector CO = ray->o - C;
            float b_half = Dot(CO, ray->d);
            float c = CO.LengthSquared() - R * R;
            float discr = b_half*b_half - c;
            if (discr <= 0.f)
                return 0.f;

            float sqrt_discr = sqrtf(discr);
            float t;
            if (lensSurface.zIntercept < lensSurface.sphereCenterZ) {
                t = -b_half - sqrt_discr;
                intersect = (*ray)(t);
                normal = Normalize(intersect - C);
            } else {
                t = -b_half + sqrt_discr;
                intersect = (*ray)(t);
                normal = Normalize(C - intersect);
            }
            assert(t > 0.f);
        }

        // check if intersection is within aperture of this lens surface
        float distToZAxisSq = intersect.x*intersect.x + intersect.y*intersect.y;
        float lensRadius = lensSurface.aperture * 0.5f;
        if (distToZAxisSq > lensRadius*lensRadius)
            return 0.f;

        if (lensSurface.refractiveRatio == 1.f)
            continue;   // ray passes through unaffected

        // calculate refracted ray
        float mu = lensSurface.refractiveRatio;
        float cos_theta_i = -Dot(ray->d, normal);
        float cos_theta_t_sq = 1.f - mu*mu*(1.f - cos_theta_i*cos_theta_i);
        if (cos_theta_t_sq < 0.f)
            return 0.f; // total internal reflection
        float gamma = mu * cos_theta_i - sqrtf(cos_theta_t_sq);

        ray->d = Normalize(mu * ray->d + gamma * normal);
        ray->o = intersect;

        assert(ray->d.z > 0.f);
    }

    CameraToWorld(*ray, ray);
    ray->d = Normalize(ray->d);

    // calculate weight of this ray
    float filmRayDirZ2 = filmRayDirZ * filmRayDirZ;
    return filmRayDirZ2 * filmRayDirZ2 * rearLensDiskArea / (filmDistance * filmDistance);
}

void  RealisticCamera::AutoFocus(Renderer * renderer, const Scene * scene, Sample * origSample) {
	// YOUR CODE HERE:
	// The current code shows how to create a new Sampler, and Film cropped to the size of the auto focus zone.
	// It then renders the film, producing rgb values.  You need to:
	//
	// 1. Modify this code so that it can adjust film plane of the camera
	// 2. Use the results of raytracing to evaluate whether the image is in focus
	// 3. Search over the space of film planes to find the best-focused plane.

	if(!autofocus)
		return;

	//for (size_t i=0; i<afZones.size(); i++) {
    for (size_t i = 0; i<1; i++) {          // JUST DO THE FIRST ONE FOR NOW!!!!!!!!!!

		AfZone & zone = afZones[i];

        RNG rng;

        // create sampler that generates 1 sample per pixel
        int xstart = Ceil2Int(film->xResolution * zone.left);
        int xend = Ceil2Int(film->xResolution * zone.right);
        int ystart = Ceil2Int(film->yResolution * zone.top);
        int yend = Ceil2Int(film->yResolution * zone.bottom);
        RandomSampler sampler(xstart, xend, ystart, yend,
            1, ShutterOpen, ShutterClose);

        int maxSamples = sampler.MaximumSampleCount();
        Sample *samples = origSample->Duplicate(maxSamples);

        // find world to camera transform at... time 0 i guess
        Transform CameraToWorld_t0;
        CameraToWorld.Interpolate(0.f, &CameraToWorld_t0);
        Transform WorldToCamera = Inverse(CameraToWorld_t0);

        // generate a ray for each sample and record its intersection with the scene
        // in camera-space coordinates as well as its distance from the camera
        struct PointDistance {
            Point p;
            float d;
            PointDistance(const Point& pp, float dd) : p(pp), d(dd) {}
            bool operator<(const PointDistance& other) const {
                return d < other.d;
            }
        };
        
        vector<PointDistance> isects;
        int sampleCount;
        while ((sampleCount = sampler.GetMoreSamples(samples, rng)) > 0) {
            for (int i = 0; i < sampleCount; ++i) {
                Ray ray;
                this->GenerateRay(samples[i], &ray);
                Intersection isect;
                if (scene->Intersect(ray, &isect)) {
                    Point p = WorldToCamera(isect.dg.p);
                    isects.push_back(PointDistance(p, (p-Point(0,0,0)).Length()));
                }
            }
        }

        // find median intersection point (we'll call it X) based on distance
        nth_element(isects.begin(), isects.begin() + isects.size() / 2, isects.end());
        Point X = isects[isects.size() / 2].p;
        
        // We'll try to set the film distance so this point is in perfect focus.
        // To do this, we'll shoot rays from this point to the camera lens and
        // record all the rays that make it to the film plane.  We then solve
        // for the optimal distance to move the film plane along the Z axis to
        // minimize the variance of the intersection points of the rays and the
        // film plane.  Basically, we're trying to minimze the size of the 
        // circle of confusion of this scene point we've chosen (point X).

        vector<Point> ps;
        vector<Vector> qs;

        delete[] samples;
	}
}

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
                                   filmDiag(filmdiag * 1.000f),
                                   filmDistance(filmdistance * 1.000f),
                                   apertureDiameter(aperture_diameter_ * 1.000f)
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

            sphereRadius *= 1.000f;
            lensZThickness *= 1.000f;
            aperture *= 1.000f;

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

    // compute the raster to camera transform
    updateRasterToCameraTransform();

    // compute rear lens disk (disk's edge coincides with edge of rear lens)
    float diskRadius, diskZ;
    getDiskOfLensSurface(lensSurfaces.back(), &diskZ, &diskRadius);
    RearLensDiskToCamera = Scale(diskRadius, diskRadius, 1.f)
                         * Translate(Vector(0.f, 0.f, diskZ));
    rearLensDiskZOffset = diskZ - lensSurfaces.back().zIntercept;

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

	printf("Read in %d AF zones from %s\n", afZones.size(), filename.c_str());
}





RealisticCamera::~RealisticCamera()
{
    
}

void RealisticCamera::updateRasterToCameraTransform() {
    float filmZ = lensSurfaces.back().zIntercept - this->filmDistance;
    float rasterDiag = sqrtf(film->xResolution*film->xResolution +
        film->yResolution*film->yResolution);
    float s = filmDiag / rasterDiag;
    RasterToCamera = Scale(-s, s, 1.f) *
        Translate(Vector(-0.5f*film->xResolution, -0.5f*film->yResolution, filmZ));
}

void RealisticCamera::getDiskOfLensSurface(const LensSurface& ls,
                                float* diskZ, float* radius) const {

    // compute rear lens disk (disk's edge coincides with edge of rear lens)
    *radius = 0.5f * ls.aperture;
    if (ls.sphereRadius == 0.f) {
        *diskZ = ls.zIntercept;
    } else {
        float r = ls.sphereRadius;
        float s = *radius;
        assert(r >= s);
        float diskZFromSphereCenter = sqrtf(r*r - s*s);
        *diskZ = (ls.zIntercept < ls.sphereCenterZ) ?
            ls.sphereCenterZ - diskZFromSphereCenter
            : ls.sphereCenterZ + diskZFromSphereCenter;
    }
}



bool RealisticCamera::traceRayThruLensSurfaces(const Ray& in, Ray* out,
                                bool frontToBack) const {
    Ray ray = in;

    // intersect ray with lens surfaces from rear to front
    for (int i = 0; i < lensSurfaces.size(); i++) {

        const LensSurface &lensSurface = frontToBack ?
            lensSurfaces[i] : lensSurfaces[lensSurfaces.size() - 1 - i];

        Point intersect;
        Vector normal;
        float R = lensSurface.sphereRadius;
        if (R == 0.f) {
            // lens surface is planar; intersect with lens plane
            if (ray.d.z == 0.f)
                return false;
            float t = (lensSurface.sphereCenterZ - ray.o.z) / ray.d.z;
            intersect = ray(t);
            normal = Vector(0.f, 0.f, -1.f);
            assert(t > 0.f);
        } else {
            // intersect with the sphere of the lens
            Point C = Point(0.f, 0.f, lensSurface.sphereCenterZ);
            Vector CO = ray.o - C;
            float b_half = Dot(CO, ray.d);
            float c = CO.LengthSquared() - R * R;
            float discr = b_half*b_half - c;
            if (discr <= 0.f)
                return false;

            float sqrt_discr = sqrtf(discr);
            float t;
            bool lensCurveTowardFront = 
                lensSurface.zIntercept > lensSurface.sphereCenterZ;

            if (lensCurveTowardFront == frontToBack) {
                t = -b_half - sqrt_discr;
                intersect = ray(t);
                normal = Normalize(intersect - C);
            } else {
                t = -b_half + sqrt_discr;
                intersect = ray(t);
                normal = Normalize(C - intersect);
            }
            assert(t > 0.f);
        }

        // check if intersection is within aperture of this lens surface
        float distToZAxisSq = intersect.x*intersect.x + intersect.y*intersect.y;
        float lensRadius = lensSurface.aperture * 0.5f;
        if (distToZAxisSq > lensRadius*lensRadius)
            return false;

        if (lensSurface.refractiveRatio == 1.f)
            continue;   // ray passes through unaffected

        // calculate refracted ray
        float mu = lensSurface.refractiveRatio;
        float cos_theta_i = -Dot(ray.d, normal);
        float cos_theta_t_sq = 1.f - mu*mu*(1.f - cos_theta_i*cos_theta_i);
        if (cos_theta_t_sq < 0.f)
            return false; // total internal reflection
        float gamma = mu * cos_theta_i - sqrtf(cos_theta_t_sq);

        ray.d = Normalize(mu * ray.d + gamma * normal);
        ray.o = intersect;

        if ((ray.d.z >= 0.f) == frontToBack)
            return false;
    }
    *out = ray;
    return true;
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

    Ray filmRay = Ray(Pras_c, Normalize(Pdisk_c - Pras_c), 0.f, INFINITY);
    
    if (!traceRayThruLensSurfaces(filmRay, ray, false))
        return 0.f;
    
    ray->o = ray->o * 0.001f;  // convert millimeters to meters
    CameraToWorld(*ray, ray);
    ray->d = Normalize(ray->d);

    // calculate weight of this ray
    float filmRayDirZ2 = filmRay.d.z * filmRay.d.z;
    float filmToDiskDistance = filmDistance - rearLensDiskZOffset;
    return filmRayDirZ2 * filmRayDirZ2 * rearLensDiskArea / (filmToDiskDistance * filmToDiskDistance);
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

    for (size_t i = 0; i<afZones.size(); i++) {

        AfZone & zone = afZones[i];

        RNG rng;
        MemoryArena arena;
        Filter * filter = new BoxFilter(.5f, .5f);
        const float crop[] = { zone.left, zone.right, zone.top, zone.bottom };
        ImageFilm sensor(film->xResolution, film->yResolution, filter, crop, "foo.exr", false);
        int xstart, xend, ystart, yend;
        sensor.GetSampleExtent(&xstart, &xend, &ystart, &yend);

        StratifiedSampler sampler(xstart, xend, ystart, yend,
            16, 16, true, ShutterOpen, ShutterClose);

        // Allocate space for samples and intersections
        int maxSamples = sampler.MaximumSampleCount();
        Sample *samples = origSample->Duplicate(maxSamples);
        RayDifferential *rays = new RayDifferential[maxSamples];
        Spectrum *Ls = new Spectrum[maxSamples];
        Spectrum *Ts = new Spectrum[maxSamples];
        Intersection *isects = new Intersection[maxSamples];

        // Get samples from _Sampler_ and update image
        int sampleCount;
        while ((sampleCount = sampler.GetMoreSamples(samples, rng)) > 0) {
            // Generate camera rays and compute radiance along rays
            for (int i = 0; i < sampleCount; ++i) {
                // Find camera ray for _sample[i]_

                float rayWeight = this->GenerateRayDifferential(samples[i], &rays[i]);
                rays[i].ScaleDifferentials(1.f / sqrtf(sampler.samplesPerPixel));


                // Evaluate radiance along camera ray

                if (rayWeight > 0.f)
                    Ls[i] = rayWeight * renderer->Li(scene, rays[i], &samples[i], rng,
                    arena, &isects[i], &Ts[i]);
                else {
                    Ls[i] = 0.f;
                    Ts[i] = 1.f;
                }

                // Issue warning if unexpected radiance value returned
                if (Ls[i].HasNaNs()) {
                    Error("Not-a-number radiance value returned "
                        "for image sample.  Setting to black.");
                    Ls[i] = Spectrum(0.f);
                } else if (Ls[i].y() < -1e-5) {
                    Error("Negative luminance value, %f, returned"
                        "for image sample.  Setting to black.", Ls[i].y());
                    Ls[i] = Spectrum(0.f);
                } else if (isinf(Ls[i].y())) {
                    Error("Infinite luminance value returned"
                        "for image sample.  Setting to black.");
                    Ls[i] = Spectrum(0.f);
                }

            }

            // Report sample results to _Sampler_, add contributions to image
            if (sampler.ReportResults(samples, rays, Ls, isects, sampleCount))
            {
                for (int i = 0; i < sampleCount; ++i)
                {

                    sensor.AddSample(samples[i], Ls[i]);

                }
            }

            // Free _MemoryArena_ memory from computing image sample values
            arena.FreeAll();
        }

        float * rgb;
        int width;
        int height;
        sensor.WriteRGB(&rgb, &width, &height, 1.f);
        // YOUR CODE HERE! The rbg contents of the image for this zone
        // are now stored in the array 'rgb'.  You can now do whatever
        // processing you wish


        //you own rgb  now so make sure to delete it:
        delete[] rgb;
        //if you want to see the output rendered from your sensor, uncomment this line (it will write a file called foo.exr)
        //sensor.WriteImage(1.f);


        delete[] samples;
        delete[] rays;
        delete[] Ls;
        delete[] Ts;
        delete[] isects;
    }
}

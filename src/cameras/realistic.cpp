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
                                   filmDiag(filmdiag),
                                   filmDistance(filmdistance),
                                   apertureDiameter(aperture_diameter_)
{
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

            if (sphereRadius == 0.f) {
                // aperture stop
                refractiveIndex = 1.f;
                if (apertureDiameter > aperture) {
                    Warning("Aperture diameter [%f] larger than max aperture [%f] in specfile.  Clamping to max.",
                        apertureDiameter, aperture);
                } else {
                    aperture = apertureDiameter;
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

    // intersect ray with lens surfaces from rear to front or front to rear
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

    CameraToWorld(*ray, ray);
    ray->d = Normalize(ray->d);

    // calculate weight of this ray
    float filmRayDirZ2 = filmRay.d.z * filmRay.d.z;
    float filmToDiskDistance = filmDistance + rearLensDiskZOffset;
    return filmRayDirZ2 * filmRayDirZ2 * rearLensDiskArea / (filmToDiskDistance * filmToDiskDistance);
}


void RealisticCamera::renderAfZone(Renderer* renderer, const Scene* scene, Sample* origSample,
                    AfZone& zone, float** rgb, int* width, int* height) {
    RNG rng;
    MemoryArena arena;
    Filter * filter = new BoxFilter(.5f, .5f);
    const float crop[] = { zone.left, zone.right, zone.top, zone.bottom };
    ImageFilm sensor(film->xResolution, film->yResolution, filter, crop, "foo.exr", false);
    int xstart, xend, ystart, yend;
    sensor.GetSampleExtent(&xstart, &xend, &ystart, &yend);

    StratifiedSampler sampler(xstart, xend, ystart, yend,
        32, 32, true, ShutterOpen, ShutterClose);

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

    sensor.WriteRGB(rgb, width, height, 1.f);


    //if you want to see the output rendered from your sensor, uncomment this line (it will write a file called foo.exr)
    //sensor.WriteImage(1.f);


    delete[] samples;
    delete[] rays;
    delete[] Ls;
    delete[] Ts;
    delete[] isects;
}


float RealisticCamera::MLofAfZone(Renderer* renderer, const Scene* scene,
    Sample* origSample, AfZone& zone, float filmDist) {

    // render the portion of the image inside the af zone using the
    // specfied film distance
    float temp = filmDistance;
    filmDistance = filmDist;
    updateRasterToCameraTransform();
    float* rgb;
    int width, height;
    renderAfZone(renderer, scene, origSample, zone, &rgb, &width, &height);
    filmDistance = temp;
    updateRasterToCameraTransform();

    // convert image to luminance
    float* I = new float[width*height];
    for (int i = 0; i < width*height; i++) {
        I[i] = 0.299f * rgb[3 * i] + 0.587f * rgb[3 * i + 1] + 0.114f * rgb[3 * i + 2];
    }

    // compute focus measure (ML for border pixels not calculated)
    const float ML_threshold = 0.0f;
    float F = 0.f;
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int center = y*width + x;
            int left = center - 1;
            int right = center + 1;
            int top = center - width;
            int bottom = center + width;
            float ML = abs(2.f*I[center] - I[left] - I[right])
                + abs(2.f*I[center] - I[top] - I[bottom]);
            if (ML > ML_threshold)
                F += ML;
        }
    }

    delete[] I;
    delete[] rgb;

    return F;
}


void  RealisticCamera::AutoFocus(Renderer * renderer, const Scene * scene, Sample * origSample) {

	if(!autofocus)
		return;

    // just use the first autofocus zone; ignore the rest.
    
    //for (size_t i = 0; i<afZones.size(); i++) {
    for (size_t n = 0; n<1; n++) {

        AfZone & zone = afZones[n];


        // find range [a,b] where focus measure is (hopefully) unimodal.
        // we're trying to get [a,b] to contain the first peak in focus measure
        // when starting the film plane all the way in the front and then moving
        // it towards the back.

        // we'll calculate the focus measure f(x) at various filmDistances x.
        // We'll start with x at half the filmDistance given by the scene description
        // and increase it by stepSize each time.

        const float PEAK_DROP_RATIO = 0.85f;
        const float VALLEY_RISE_RATIO = 1.f / PEAK_DROP_RATIO;
        const float stepSize = filmDistance / 15.f;

        float a = 0.5f * filmDistance;
        float b;

        vector<float> f_xs; // f(x)s ordered from smaller to larger filmDistances

        float f_x;
        float x = a;

        // go past first valley
        float min_f_x = INFINITY;   // min f(x) after most recent peak
        int min_f_x_index;
        int i = 0;
        while (true) {
            f_x = MLofAfZone(renderer, scene, origSample, zone, x);
            printf("Focus measure at filmDistance %.2f: %.2f\n", x, f_x);
            if (f_x < min_f_x) {
                min_f_x_index = i;
                min_f_x = f_x;
            } else if (f_x > min_f_x * VALLEY_RISE_RATIO) {
                break;
            }
            f_xs.push_back(f_x);
            x += stepSize;
            i++;
            if (x > filmDistance * 4.f) {
                printf("Failed to find valley in focus measure! Autofocus will not occur.\n");
                return;
            }
        }

        // go past first peak after that
        float max_f_x = f_x;
        int max_f_x_index = i;

        f_xs.push_back(f_x);
        x += stepSize;
        i++;

        while (true) {
            f_x = MLofAfZone(renderer, scene, origSample, zone, x);
            printf("Focus measure at filmDistance %.2f: %.2f\n", x, f_x);
            if (f_x > max_f_x) {
                max_f_x_index = i;
                max_f_x = f_x;
            } else if (f_x < PEAK_DROP_RATIO * max_f_x) {
                break;
            }
            f_xs.push_back(f_x);
            x += stepSize;  // 10mm intervals
            i++;
            if (x > filmDistance * 4.f) {
                printf("Failed to find peak in focus measure! Autofocus will not occur.\n");
                return;
            }
        }
        printf("Focus peak found!\n");
        b = x;
        // search backwards thru the recorded f(x)'s to find the first one where
        // f(x) < max f(x) so that everything left of x can be discarded.
        for (i = max_f_x_index - 1; i > min_f_x_index; i--) {
            if (f_xs[i] < PEAK_DROP_RATIO * max_f_x)
                break;
        }
        a += i * stepSize;

        assert(f_xs[min_f_x_index] == min_f_x);
        assert(f_xs[max_f_x_index] == max_f_x);
        
        /*
        const float PEAK_DROP_RATIO = 0.85f;
        const float VALLEY_RISE_RATIO = 1.f / PEAK_DROP_RATIO;

        float a = 0.5f * filmDistance;
        float b = 2.f * filmDistance;
        float stepSize = filmDistance / 15.f;

        vector<float> f_xs;
        for (float x = a; x <= b; x += stepSize) {
            float f_x = MLofAfZone(renderer, scene, origSample, zone, x);
            f_xs.push_back(f_x);
            printf("Focus measure at filmDistance %.2f: %.2f\n", x, f_x);
        }

        float maxPeak = 0.f;
        int maxPeakIndex;
        for (int i = 1; i < f_xs.size() - 1; i++) {
            if (f_xs[i - 1] < PEAK_DROP_RATIO * f_xs[i] 
                && f_xs[i] * PEAK_DROP_RATIO > f_xs[i + 1]
                && f_xs[i] > maxPeak) {
                maxPeak = f_xs[i];
                maxPeakIndex = i;
            }
        }
        a += (maxPeakIndex - 1) * stepSize;
        b = a + 2 * stepSize;
        */

        // Now we have an interval [a,b] that contains a peak of the focus measure.
        // We'll now find the x that maximizes f(x) within that interval using
        // golden ratio section search

        const float EPSILON = 2.f;
        const float alpha = 0.5f * (3.f - sqrtf(5));
        
        printf("\nfilmDistance search range: [%.2f, %.2f]\n", a, b);
        float x0;
        float x1;
        float f_x0;
        float f_x1;
        bool f_x0_unknown = true;
        bool f_x1_unknown = true;
        while (b - a >= EPSILON) {
            // calculate the f(x) that's not being reused from last iteration
            if (f_x0_unknown) {
                x0 = a + alpha * (b - a);
                f_x0 = MLofAfZone(renderer, scene, origSample, zone, x0);
            }
            if (f_x1_unknown) {
                x1 = a + (1 - alpha) * (b - a);
                f_x1 = MLofAfZone(renderer, scene, origSample, zone, x1);
            }
            // discard a third of the interval
            if (f_x0 < f_x1) {
                a = x0;
                x0 = x1;
                f_x0 = f_x1;
                f_x0_unknown = false;
                f_x1_unknown = true;
            } else if (f_x0 > f_x1) {
                b = x1;
                x1 = x0;
                f_x1 = f_x0;
                f_x0_unknown = true;
                f_x1_unknown = false;
            } else {
                // clip search range by 5% on each end; hopefully get
                // different f(x)s next iteration
                float c = 0.05f * (b - a);
                a += c;
                b -= c;
                f_x0_unknown = true;
                f_x1_unknown = true;
            }
            printf("filmDistance search range: [%.2f, %.2f]\n", a, b);
        }

        filmDistance = 0.5f * (a + b);
        updateRasterToCameraTransform();
        printf("filmDistance set to %.2f\n\n", filmDistance);
    }
}

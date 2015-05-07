#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CAMERAS_REALISTIC_H
#define PBRT_CAMERAS_REALISTIC_H

#include "pbrt.h"
#include "camera.h"
#include "film.h"

// Example representation of an autofocus zone.
class AfZone {
	public:
	  // from the data file
	  float left, right;
	  float top, bottom;
	  int xres,yres;
};

class LensSurface {
public:
    float zIntercept;
    float sphereRadius;     // 0 if is flat plane
    float sphereCenterZ;     // plane Z intercept if flat plane
    float refractiveRatio;  // n_back/n_front
    float aperture;         // diameter of surface
};

class RealisticCamera : public Camera {
public:
   RealisticCamera(const AnimatedTransform &cam2world,
      float hither, float yon, float sopen,
      float sclose, float filmdistance, float aperture_diameter,
      const string &specfile,
	  const string &autofocusfile,
      float filmdiag,
	  Film *film);
   ~RealisticCamera();
   float GenerateRay(const CameraSample &sample, Ray *) const;
   void  AutoFocus(Renderer * renderer, const Scene * scene, Sample * origSample);
   void  ParseAfZones(const string& filename);

private:
   void updateRasterToCameraTransform();
   bool traceRayThruLensSurfaces(const Ray& in, Ray* out, bool frontToBack) const;
   void getDiskOfLensSurface(const LensSurface& ls, float* diskZ, float* radius) const;

   bool  autofocus;
   vector<AfZone> afZones;
   float ShutterOpen;
   float ShutterClose;
   Film * film;

   float filmDiag;
   float filmDistance;
   float apertureDiameter;
   vector<LensSurface> lensSurfaces;    // ordered front to back
   
   float rearLensDiskArea;
   float rearLensDiskZOffset;       // equal to diskZ - rearLens.zIntercept

   Transform RasterToCamera;
   Transform RearLensDiskToCamera; // transforms point in unit circle to point on rear lens disk
};

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film);

#endif

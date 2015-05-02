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
    float sphereRadius;     // 0 if is flat plane
    float sphereCenter;     // plane Z intercept if flat plane
    float refractiveRatio;  // n_front/n_back
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
   bool  autofocus;
   vector<AfZone> afZones;
   float ShutterOpen;
   float ShutterClose;
   Film * film;

   vector<LensSurface> lensSurfaces;    // ordered front to back
};

RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film);

#endif

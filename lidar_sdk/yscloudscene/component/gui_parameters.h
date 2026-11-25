#ifndef YS_SCENE_GUI_PARAMETERS_H_
#define YS_SCENE_GUI_PARAMETERS_H_

#include "color_types.hpp"

namespace ys_scene {
class YsGui
{
public:

	//! GUI parameters
	struct ParamStruct
	{
		//! Light diffuse color (RGBA)
		Rgbaf lightDiffuseColor;
		//! Light ambient color (RGBA)
		Rgbaf lightAmbientColor;
		//! Light specular color (RGBA)
		Rgbaf lightSpecularColor;
		//! Double sided light
		bool lightDoubleSided;

		//! Default mesh diffuse color (front)
		Rgbaf meshFrontDiff;
		//! Default mesh diffuse color (back)
		Rgbaf meshBackDiff;
		//! Default mesh specular color
		Rgbaf meshSpecular;

		//! Default text color
		Rgba textDefaultCol;
		//! Default 3D points color
		Rgba pointsDefaultCol;
		//! Background color
		Rgbub backgroundCol;
		//! Labels background color
		Rgba labelBackgroundCol;
		//! Labels marker color
		Rgba labelMarkerCol;
		//! Bounding-boxes color
		Rgba bbDefaultCol;
		
		//! Use background gradient
		bool drawBackgroundGradient;
		//! Decimate meshes when moved
		bool decimateMeshOnMove;
		//! Min mesh size for decimation
		unsigned minLoDMeshSize;
		//! Decimate clouds when moved
		bool decimateCloudOnMove;
		//! Min cloud size for decimation
		unsigned minLoDCloudSize;
		//! Display cross in the middle of the screen
		bool displayCross;
		//! Whether to use VBOs for faster display
		bool useVBOs;

		//! Label marker size
		unsigned labelMarkerSize;

		//! Color scale option: show histogram next to color ramp
		bool colorScaleShowHistogram;
		//! Whether to use shader for color scale display (if available) or not
		bool colorScaleUseShader;
		//! Whether shader for color scale display is available or not
		bool colorScaleShaderSupported;
		//! Color scale ramp width (for display)
		unsigned colorScaleRampWidth;

		//! Default displayed font size
		unsigned defaultFontSize;
		//! Label font size
		unsigned labelFontSize;
		//! Displayed numbers precision
		unsigned displayedNumPrecision;
		//! Labels background opcaity
		unsigned labelOpacity;

		//! Zoom speed (1.0 by default)
		double zoomSpeed;

		//! Octree computation (for picking) behaviors
		enum ComputeOctreeForPicking { ALWAYS = 0, ASK_USER = 1, NEVER = 2 };

		//! Octree computation (for picking) behavior
		ComputeOctreeForPicking autoComputeOctree;

		//! Whether to draw rounded points (slower) or not
		bool drawRoundedPoints;

		//! Whether to pick objects by single clicking in the GUI 
		//! can be slow with large point clouds/large number of objects
		bool singleClickPicking;

		//! Picking cursor
		Qt::CursorShape pickingCursorShape;

		//! Default constructor
		ParamStruct();

		//! Resets parameters to default values
		void reset();

		//! Loads from persistent DB
		void fromPersistentSettings();

		//! Saves to persistent DB
		void toPersistentSettings() const;

		//! Returns whether a given parameter is already defined in persistent settings or not
		/** \param paramName the corresponding attribute name
		**/
		bool isInPersistentSettings(QString paramName) const;
	};

	//! Returns the stored values of each parameter.
	static const ParamStruct& Parameters();

	//! Sets GUI parameters
	static void Set(const ParamStruct& params);

	//! Release unique instance (if any)
	static void ReleaseInstance();

protected:

	//! Parameters set
	ParamStruct params;

};

} // namespace ys_scene

#endif // YS_SCENE_GUI_PARAMETERS_H_
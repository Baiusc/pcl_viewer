#include "gui_parameters.h"

//Qt
#include <QSettings>

#include <ys_singleton.h>

//System
#include <string.h>

namespace ys_scene {
//! Unique instance of YsGui
static YsSingleton<YsGui> s_gui;

const YsGui::ParamStruct& YsGui::Parameters()
{
	if (!s_gui.instance_)
	{
		s_gui.instance_ = new YsGui();
		s_gui.instance_->params.fromPersistentSettings();
	}

	return s_gui.instance_->params;
}

void YsGui::ReleaseInstance()
{
	s_gui.Release();
}

void YsGui::Set(const ParamStruct& params)
{
	if (!s_gui.instance_)
	{
		s_gui.instance_ = new YsGui();
		s_gui.instance_->params.fromPersistentSettings();
	}

	s_gui.instance_->params = params;
}

YsGui::ParamStruct::ParamStruct()
{
	reset();
}

void YsGui::ParamStruct::reset()
{
	lightAmbientColor = night;
	lightSpecularColor = darker;
	lightDiffuseColor = bright;
	meshFrontDiff = defaultMeshFrontDiff;
	meshBackDiff = defaultMeshBackDiff;
	meshSpecular = middle;
	pointsDefaultCol = defaultColor;
	textDefaultCol = defaultColor;
	backgroundCol = defaultBkgColor;
	labelBackgroundCol = defaultLabelBkgColor;
	labelMarkerCol = defaultLabelMarkerColor;
	bbDefaultCol = yellow;

	lightDoubleSided = true;
	drawBackgroundGradient = true;
	drawRoundedPoints = false;
	decimateMeshOnMove = true;
	minLoDMeshSize = 2500000;
	decimateCloudOnMove = true;
	minLoDCloudSize = 50000000;
	useVBOs = true;
	displayCross = true;
	pickingCursorShape = Qt::CrossCursor;

	labelMarkerSize = 5;

	colorScaleShowHistogram = true;
	colorScaleUseShader = false;
	colorScaleShaderSupported = false;
	colorScaleRampWidth = 50;

#ifdef Q_OS_MAC
	defaultFontSize = 12;
	labelFontSize = 10;
#else
	defaultFontSize = 10;
	labelFontSize = 8;
#endif

	displayedNumPrecision = 6;
	labelOpacity = 75;

	zoomSpeed = 1.0;

	autoComputeOctree = ASK_USER;

	singleClickPicking = true;
}

static int c_fColorArraySize = sizeof(float) * 4;
static int c_ub3ColorArraySize = sizeof(unsigned char) * 3;
static int c_ub4ColorArraySize = sizeof(unsigned char) * 4;

void YsGui::ParamStruct::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	lightAmbientColor = Rgbaf(reinterpret_cast<float*>(settings.value("lightAmbientColor", QByteArray::fromRawData((const char*)darkest.rgba, c_fColorArraySize)).toByteArray().data()));
	lightSpecularColor = Rgbaf(reinterpret_cast<float*>(settings.value("lightSpecularColor", QByteArray::fromRawData((const char*)darker.rgba, c_fColorArraySize)).toByteArray().data()));
	lightDiffuseColor = Rgbaf(reinterpret_cast<float*>(settings.value("lightDiffuseColor", QByteArray::fromRawData((const char*)bright.rgba, c_fColorArraySize)).toByteArray().data()));
	meshFrontDiff = Rgbaf(reinterpret_cast<float*>(settings.value("meshFrontDiff", QByteArray::fromRawData((const char*)defaultMeshFrontDiff.rgba, c_fColorArraySize)).toByteArray().data()));
	meshBackDiff = Rgbaf(reinterpret_cast<float*>(settings.value("meshBackDiff", QByteArray::fromRawData((const char*)defaultMeshBackDiff.rgba, c_fColorArraySize)).toByteArray().data()));
	meshSpecular = Rgbaf(reinterpret_cast<float*>(settings.value("meshSpecular", QByteArray::fromRawData((const char*)middle.rgba, c_fColorArraySize)).toByteArray().data()));
	pointsDefaultCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("pointsDefaultRGBAColor", QByteArray::fromRawData((const char*)defaultColor.rgba, c_ub4ColorArraySize)).toByteArray().data()));
	textDefaultCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("textDefaultRGBAColor", QByteArray::fromRawData((const char*)defaultColor.rgba, c_ub4ColorArraySize)).toByteArray().data()));
	backgroundCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("backgroundColor", QByteArray::fromRawData((const char*)defaultBkgColor.rgb, c_ub3ColorArraySize)).toByteArray().data()));
	labelBackgroundCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("labelBackgroundRGBAColor", QByteArray::fromRawData((const char*)defaultLabelBkgColor.rgba, c_ub4ColorArraySize)).toByteArray().data()));
	labelMarkerCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("labelMarkerRGBAColor", QByteArray::fromRawData((const char*)defaultLabelMarkerColor.rgba, c_ub4ColorArraySize)).toByteArray().data()));
	bbDefaultCol = Rgbaub(reinterpret_cast<unsigned char*>(settings.value("bbDefaultRGBAColor", QByteArray::fromRawData((const char*)yellow.rgba, c_ub4ColorArraySize)).toByteArray().data()));

	lightDoubleSided = settings.value("lightDoubleSided", true).toBool();
	drawBackgroundGradient = settings.value("backgroundGradient", true).toBool();
	drawRoundedPoints = settings.value("drawRoundedPoints", false).toBool();
	decimateMeshOnMove = settings.value("meshDecimation", true).toBool();
	minLoDMeshSize = settings.value("minLoDMeshSize", 2500000).toUInt();
	decimateCloudOnMove = settings.value("cloudDecimation", true).toBool();
	minLoDCloudSize = settings.value("minLoDCloudSize", 50000000).toUInt();
	useVBOs = settings.value("useVBOs", true).toBool();
	displayCross = settings.value("crossDisplayed", true).toBool();
	labelMarkerSize = static_cast<unsigned>(std::max(0, settings.value("labelMarkerSize", 5).toInt()));
	colorScaleShowHistogram = settings.value("colorScaleShowHistogram", true).toBool();
	colorScaleUseShader = settings.value("colorScaleUseShader", false).toBool();
	//colorScaleShaderSupported	= not saved
	colorScaleRampWidth = static_cast<unsigned>(std::max(0, settings.value("colorScaleRampWidth", 50).toInt()));
	defaultFontSize = static_cast<unsigned>(std::max(0, settings.value("defaultFontSize", 10).toInt()));
	labelFontSize = static_cast<unsigned>(std::max(0, settings.value("labelFontSize", 8).toInt()));
	displayedNumPrecision = static_cast<unsigned>(std::max(0, settings.value("displayedNumPrecision", 6).toInt()));
	labelOpacity = static_cast<unsigned>(std::max(0, settings.value("labelOpacity", 75).toInt()));
	zoomSpeed = settings.value("zoomSpeed", 1.0).toDouble();
	autoComputeOctree = static_cast<ComputeOctreeForPicking>(settings.value("autoComputeOctree", ASK_USER).toInt());
	singleClickPicking = settings.value("singleClickPicking", true).toBool();
	pickingCursorShape = static_cast<Qt::CursorShape>(settings.value("cursorShape", static_cast<int>(Qt::CrossCursor)).toInt());

	settings.endGroup();
}

void YsGui::ParamStruct::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("OpenGL");

	settings.setValue("lightDiffuseColor", QByteArray((const char*)lightDiffuseColor.rgba, c_fColorArraySize));
	settings.setValue("lightAmbientColor", QByteArray((const char*)lightAmbientColor.rgba, c_fColorArraySize));
	settings.setValue("lightSpecularColor", QByteArray((const char*)lightSpecularColor.rgba, c_fColorArraySize));
	settings.setValue("meshFrontDiff", QByteArray((const char*)meshFrontDiff.rgba, c_fColorArraySize));
	settings.setValue("meshBackDiff", QByteArray((const char*)meshBackDiff.rgba, c_fColorArraySize));
	settings.setValue("meshSpecular", QByteArray((const char*)meshSpecular.rgba, c_fColorArraySize));
	settings.setValue("pointsDefaultRGBAColor", QByteArray((const char*)pointsDefaultCol.rgba, c_ub4ColorArraySize));
	settings.setValue("textDefaultRGBAColor", QByteArray((const char*)textDefaultCol.rgba, c_ub4ColorArraySize));
	settings.setValue("backgroundColor", QByteArray((const char*)backgroundCol.rgb, c_ub3ColorArraySize));
	settings.setValue("labelBackgroundRGBAColor", QByteArray((const char*)labelBackgroundCol.rgba, c_ub4ColorArraySize));
	settings.setValue("labelMarkerRGBAColor", QByteArray((const char*)labelMarkerCol.rgba, c_ub4ColorArraySize));
	settings.setValue("bbDefaultRGBAColor", QByteArray((const char*)bbDefaultCol.rgba, c_ub4ColorArraySize));
	settings.setValue("backgroundGradient", drawBackgroundGradient);
	settings.setValue("drawRoundedPoints", drawRoundedPoints);
	settings.setValue("meshDecimation", decimateMeshOnMove);
	settings.setValue("minLoDMeshSize", minLoDMeshSize);
	settings.setValue("cloudDecimation", decimateCloudOnMove);
	settings.setValue("minLoDCloudSize", minLoDCloudSize);
	settings.setValue("useVBOs", useVBOs);
	settings.setValue("crossDisplayed", displayCross);
	settings.setValue("labelMarkerSize", labelMarkerSize);
	settings.setValue("colorScaleShowHistogram", colorScaleShowHistogram);
	settings.setValue("colorScaleUseShader", colorScaleUseShader);
	//settings.setValue("colorScaleShaderSupported", not saved);
	settings.setValue("colorScaleRampWidth", colorScaleRampWidth);
	settings.setValue("defaultFontSize", defaultFontSize);
	settings.setValue("labelFontSize", labelFontSize);
	settings.setValue("displayedNumPrecision", displayedNumPrecision);
	settings.setValue("labelOpacity", labelOpacity);
	settings.setValue("zoomSpeed", zoomSpeed);
	settings.setValue("autoComputeOctree", static_cast<int>(autoComputeOctree));
	settings.setValue("singleClickPicking", singleClickPicking);
	settings.setValue("cursorShape", static_cast<int>(pickingCursorShape));

	settings.endGroup();
}

bool YsGui::ParamStruct::isInPersistentSettings(QString paramName) const
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	return settings.contains(paramName);
}
} // namespace ys_scene

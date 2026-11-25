#ifndef YS_SCENE_COLOR_TYPES_HPP_
#define YS_SCENE_COLOR_TYPES_HPP_

#include <QColor>

namespace ys_scene {

using ColorCompType = unsigned char;

	//! Color scale element: one value + one color
class ColorScaleElement
{
public:
	
	//! Default constructor
	ColorScaleElement() : relativepos_(0.0), color_(Qt::black) {}

	//! Constructor from a (relative) position and a color
	ColorScaleElement(double relativePos, const QColor& color) : relativepos_(relativePos), color_(color) {}

	//! Sets associated value (relative to scale boundaries)
	/** \param pos relative position (always between 0.0 and 1.0!)
	**/
	inline void SetRelativePos(double pos) { relativepos_ = pos; }
	//! Returns step position (relative to scale boundaries)
	/** \return relative position (always between 0.0 and 1.0!)
	**/
	inline double GetRelativePos() const { return relativepos_; }

	//! Sets color
	inline void SetColor(const QColor& color) { color_ = color; }
	//! Returns color
	inline const QColor& GetColor() const { return color_; }

	//! Comparison operator between two color scale elements
	inline static bool IsSmaller(const ColorScaleElement& e1, const ColorScaleElement& e2)
	{
		return e1.GetRelativePos() < e2.GetRelativePos();
	}

protected:
	
	//! Step (relative) position
	/** Must always be between 0.0 and 1.0!
	**/
	double relativepos_;
	//! Color
	QColor color_;
};

//! RGB color structure
template <typename Type> 
class RgbTpl{
public:

	//! 3-tuple as a union
	union
	{
		struct
		{
			Type r, g, b;
		};
		Type rgb[3];
	};

	//! Default constructor
	/** Inits color to (0,0,0).
	**/
	constexpr inline RgbTpl() : r(0), g(0), b(0) {}

	//! Constructor from a triplet of r,g,b values
	explicit constexpr inline RgbTpl(Type red, Type green, Type blue) : r(red), g(green), b(blue) {}

	//! Constructor from an array of 3 values
	explicit constexpr inline RgbTpl(const Type col[3]) : r(col[0]), g(col[1]), b(col[2]) {}

	//! Comparison operator
	inline bool operator != (const RgbTpl<Type>& t) const { return (r != t.r || g != t.g || b != t.b); }
};

// !3 components, float type
using Rgbf = RgbTpl<float>;
//! 3 components, unsigned byte type
using Rgbub = RgbTpl<unsigned char>;
//! 3 components, default type
using Rgb = RgbTpl<ColorCompType>;

//! RGBA color structure
template <class Type> class RgbaTpl
{
public:

	// 4-tuple values as a union
	union
	{
		struct
		{
			Type r, g, b, a;
		};
		Type rgba[4];
	};

	//! Default constructor
	/** Inits color to (0,0,0,0).
	**/
	constexpr inline RgbaTpl() : r(0), g(0), b(0), a(0) {}

	//! Constructor from a triplet of r,g,b values and a transparency value
	explicit constexpr inline RgbaTpl(Type red, Type green, Type blue, Type alpha) : r(red), g(green), b(blue), a(alpha) {}

	//! RgbaTpl from an array of 4 values
	explicit constexpr inline RgbaTpl(const Type col[4]) : r(col[0]), g(col[1]), b(col[2]), a(col[3]) {}
	//! RgbaTpl from an array of 3 values and a transparency value
	explicit constexpr inline RgbaTpl(const Type col[3], Type alpha) : r(col[0]), g(col[1]), b(col[2]), a(alpha) {}

	//! Copy constructor
	inline RgbaTpl(const RgbTpl<Type>& c, Type alpha) : r(c.r), g(c.g), b(c.b), a(alpha) {}

	//! Cast operator
	inline operator RgbTpl<Type>() const { return RgbTpl<Type>(rgba); }
	//! Cast operator (const version)
	//inline operator const Type*() const { return rgba; }

	//! Comparison operator
	inline bool operator != (const RgbaTpl<Type>& t) const { return (r != t.r || g != t.g || b != t.b || a != t.a); }
};

//! 4 components, float type
using Rgbaf = RgbaTpl<float>;
//! 4 components, unsigned byte type
using Rgbaub = RgbaTpl<unsigned char>;
//! 4 components, default type
using Rgba = RgbaTpl<ColorCompType>;

constexpr unsigned max_steps_ = 1024;
constexpr unsigned min_steps_ = 2;
constexpr ColorCompType MAX = 255;
constexpr int MAXi = 255;

// Predefined colors (default type)
constexpr Rgb whiteRGB(MAX, MAX, MAX);
constexpr Rgb lightGreyRGB(static_cast<ColorCompType>(MAXi * 8 / 10), static_cast<ColorCompType>(MAXi * 8 / 10), static_cast<ColorCompType>(MAXi * 8 / 10));
constexpr Rgb darkGreyRGB(MAX / 2, MAX / 2, MAX / 2);
constexpr Rgb redRGB(MAX, 0, 0);
constexpr Rgb greenRGB(0, MAX, 0);
constexpr Rgb blueRGB(0, 0, MAX);
constexpr Rgb blueCCRGB(0, static_cast<ColorCompType>(MAXi * 7 / 10), MAX);
constexpr Rgb darkBlueRGB(0, 0, MAX / 2);
constexpr Rgb magentaRGB(MAX, 0, MAX);
constexpr Rgb cyanRGB(0, MAX, MAX);
constexpr Rgb orangeRGB(MAX, MAX / 2, 0);
constexpr Rgb blackRGB(0, 0, 0);
constexpr Rgb yellowRGB(MAX, MAX, 0);

// Predefined colors (default type)
constexpr Rgba white(whiteRGB.r, whiteRGB.g, whiteRGB.b, MAX);
constexpr Rgba lightGrey(lightGreyRGB.r, lightGreyRGB.g, lightGreyRGB.b, MAX);
constexpr Rgba darkGrey(darkGreyRGB.r, darkGreyRGB.g, darkGreyRGB.b, MAX);
constexpr Rgba red(redRGB.r, redRGB.g, redRGB.b, MAX);
constexpr Rgba green(greenRGB.r, greenRGB.g, greenRGB.b, MAX);
constexpr Rgba blue(blueRGB.r, blueRGB.g, blueRGB.b, MAX);
constexpr Rgba blueCC(blueCCRGB.r, blueCCRGB.g, blueCCRGB.b, MAX);
constexpr Rgba darkBlue(darkBlueRGB.r, darkBlueRGB.g, darkBlueRGB.b, MAX);
constexpr Rgba magenta(magentaRGB.r, magentaRGB.g, magentaRGB.b, MAX);
constexpr Rgba cyan(cyanRGB.r, cyanRGB.g, cyanRGB.b, MAX);
constexpr Rgba orange(orangeRGB.r, orangeRGB.g, orangeRGB.b, MAX);
constexpr Rgba black(blackRGB.r, blackRGB.g, blackRGB.b, MAX);
constexpr Rgba yellow(yellowRGB.r, yellowRGB.g, yellowRGB.b, MAX);

// Predefined materials (float)
constexpr Rgbaf bright(1.00f, 1.00f, 1.00f, 1.00f);
constexpr Rgbaf lighter(0.83f, 0.83f, 0.83f, 1.00f);
constexpr Rgbaf light(0.66f, 0.66f, 0.66f, 1.00f);
constexpr Rgbaf middle(0.50f, 0.50f, 0.50f, 1.00f);
constexpr Rgbaf dark(0.34f, 0.34f, 0.34f, 1.00f);
constexpr Rgbaf darker(0.17f, 0.17f, 0.17f, 1.00f);
constexpr Rgbaf darkest(0.08f, 0.08f, 0.08f, 1.00f);
constexpr Rgbaf night(0.00f, 0.00f, 0.00f, 1.00F);
constexpr Rgbaf defaultMeshFrontDiff(0.00f, 0.90f, 0.27f, 1.00f);
constexpr Rgbaf defaultMeshBackDiff(0.27f, 0.90f, 0.90f, 1.00f);

// Default foreground color (unsigned byte)
constexpr Rgbub defaultBkgColor(10, 102, 151);		// dark blue
constexpr Rgba  defaultColor(MAX, MAX, MAX, MAX);	// white
constexpr Rgba  defaultLabelBkgColor(MAX, MAX, MAX, MAX);	// white
constexpr Rgba  defaultLabelMarkerColor(MAX, 0, MAX, MAX);	// magenta

} // namespace ys_scene 
#endif // !YS_SCENE_COLOR_TYPES_HPP_

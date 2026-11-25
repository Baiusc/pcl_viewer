#ifndef YS_SCENE_SCALAR_FIELD_HPP_
#define YS_SCENE_SCALAR_FIELD_HPP_

#include <assert.h>
#include <algorithm>

namespace ys_scene{
using ScalarType = float;
class Range
{
public:

	//! Default constructor
	Range() : m_min(0), m_start(0), m_stop(0), m_max(0), m_range(1) {}

	//getters
	inline ScalarType min()			const { return m_min;		}
	inline ScalarType start()		const { return m_start;		}
	inline ScalarType stop()		const { return m_stop;		}
	inline ScalarType max()			const { return m_max;		}
	inline ScalarType range()		const { return m_range;		}
	inline ScalarType maxRange()	const { return m_max-m_min; }

	//setters
	void setBounds(ScalarType minVal, ScalarType maxVal, bool resetStartStop = true)
	{
		assert(minVal <=maxVal);
		m_min = minVal; m_max = maxVal;
		if (resetStartStop)
		{
			m_start = m_min;
			m_stop = m_max;
		}
		else
		{
			m_start = inbound(m_start);
			m_stop = inbound(m_stop);
		}
		updateRange();
	}

	inline void setStart(ScalarType value) { m_start = inbound(value); if (m_stop < m_start) m_stop = m_start; updateRange(); }
	inline void setStop(ScalarType value) { m_stop = inbound(value); if (m_stop < m_start) m_start = m_stop; updateRange(); }
	
	//! Returns the nearest inbound value
	inline ScalarType inbound(ScalarType val) const { return (val < m_min ?  m_min : (val > m_max ? m_max : val)); }
	//! Returns whether a value is inbound or not
	inline bool isInbound(ScalarType val) const { return (val >= m_min && val <= m_max); }
	//! Returns whether a value is inside range or not
	inline bool isInRange(ScalarType val) const { return (val >= m_start && val <= m_stop); }

protected:

	//! Updates actual range
	inline void updateRange() { m_range = std::max(m_stop - m_start, std::numeric_limits<ScalarType>::epsilon()); }

	ScalarType m_min;		/**< Minimum value **/
	ScalarType m_start;		/**< Current start value (in [min,max]) **/
	ScalarType m_stop;		/**< Current stop value (in [min,max]) **/
	ScalarType m_max;		/**< Minimum value **/
	ScalarType m_range;		/**< Actual range: start-stop (but can't be ZERO!) **/
};
}; // namepace ys_scene

#endif // YS_SCENE_SCALAR_FIELD_HPP_
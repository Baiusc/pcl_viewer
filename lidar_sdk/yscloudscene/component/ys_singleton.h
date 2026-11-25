#ifndef YS_SCENE_SINGLETON_H_
#define YS_SCENE_SINGLETON_H_

namespace ys_scene {

template<class T> 
struct YsSingleton
{
	//! Default constructor
	YsSingleton() : instance_(nullptr) {}
	//! Destructor
	~YsSingleton() { Release(); }
	//! Releases the current instance
	inline void Release() { if (instance_) { delete instance_; instance_ = nullptr; } }
	
	//! Current instance
	T* instance_;
};

} // namespace ys_scene 

#endif // YS_SCENE_SINGLETON_H_
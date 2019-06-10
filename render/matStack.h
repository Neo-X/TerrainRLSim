#ifndef _MatStack_
#define _MatStack_

// #include "glm/glm/glm.hpp"
/// glm::translate, glm::rotate, glm::scale, glm::perspective
// #include "glm/glm/gtc/matrix_transform.hpp"
#include "util/MathUtil.h"

// NOT THE MOST EFFICIENT IMPLEMENTATION

namespace Util {

	enum MAT_STACK_MODE
	{
		PROJECTION_MAT = 0,
		MODELVIEW_MAT
	};
	class ModelviewStack {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		int _MAX;
		// int _top;
		// int _topView;
		MAT_STACK_MODE _mMode;

		std::vector<tMatrix> _items;
		std::vector<tMatrix> _viewMat;
		// glm::mat4* _items;
		// glm::mat4* _viewMat;


	public:
		ModelviewStack(int size)
			// :_viewMat(size), _items (size)
		{
			_MAX = size;
			// _top = -1;
			// _topView = -1;
			setMatrixMode(PROJECTION_MAT);
			Push(tMatrix::Identity());
			setMatrixMode(MODELVIEW_MAT);
			Push(tMatrix::Identity());
			// _items = new glm::mat4[_MAX];
			// _viewMat = new glm::mat4[_MAX];
		}

		~ModelviewStack()
		{
			/// Potential memory leak
			// delete[] _items;
			// delete[] _viewMat;
			// _viewMat.clear();
			// _items.clear();
		}

		void setMatrixMode(MAT_STACK_MODE mode)
		{
			// _top = -1;
			_mMode = mode;
		}
		void LoadIdentity(void) {
			// _top = -1;
			Load(tMatrix::Identity());
			// Push(glm::mat4(1.0));
		}
		void Load(const tMatrix & m) {
			// _top = -1;
			// Push(m);
			if (_mMode == MODELVIEW_MAT)
				_items[size()] = m;
			else
				_viewMat[sizeView()] = m;
		}
		void Push(const tMatrix & m){
			if (Full()){
				std::cout << "Stack Full!" << std::endl;
				return;
			}

			if (_mMode == MODELVIEW_MAT)
				_items.push_back(m);
			else
				_viewMat.push_back(m);

		}

		const tMatrix Top() {
			if (Empty()){
				std::cout << "Stack Empty!" << std::endl;
				return(tMatrix::Identity());
			}

			// return(_viewMat*_items[size()]);
			return(_items[size()]);
		}
		const tMatrix View() {
			// return _viewMat;
			return(_viewMat[sizeView()]);
		}

		int size()
		{
			return int(_items.size()) - 1;
		}
		int sizeView()
		{
			return int(_viewMat.size()) - 1;
		}

		int Empty()
		{
			if (_mMode == MODELVIEW_MAT)
				return size() == -1;
			else
				return sizeView() == -1;
		}

		int Full()
		{
			if (_mMode == MODELVIEW_MAT)
				return size() + 1 == _MAX;
			else
				return sizeView() + 1 == _MAX;
		}

		void Rotate(float angle, const tVector & axis)
		{
			if (size() < 0) return;
			if (_mMode == MODELVIEW_MAT)
			{
				// _items[size()] = glm::rotate(_items[size()], angle, axis);
				_items[size()] = _items[size()] * cMathUtil::RotateMat(axis, angle);
			}
			else
			{
				// _viewMat[sizeView()] = glm::rotate(_viewMat[sizeView()], angle, axis);
				_viewMat[sizeView()] = _viewMat[sizeView()] * cMathUtil::RotateMat(axis, angle);
			}
		}
		void Translate(const tVector & offset)
		{
			if (size() < 0) return;
			if (_mMode == MODELVIEW_MAT)
			{
				// _items[size()] = glm::translate(_items[size()], offset);
				_items[size()] = _items[size()] * cMathUtil::TranslateMat(offset);
			}
			else
			{
				// _viewMat[sizeView()] = glm::translate(_viewMat[sizeView()], offset);
				_viewMat[sizeView()] = _viewMat[sizeView()] * cMathUtil::TranslateMat(offset);
			}
		}
		void Scale(const tVector & factors)
		{
			if (size() < 0) return;
			if (_mMode == MODELVIEW_MAT)
			{
				// _items[size()] = glm::scale(_items[size()], factors);
				_items[size()] = _items[size()] * cMathUtil::ScaleMat(factors);
			}
			else
			{
				// _viewMat[sizeView()] = glm::scale(_viewMat[sizeView()], factors);
				_viewMat[sizeView()] = _viewMat[sizeView()] * cMathUtil::ScaleMat(factors);
			}
		}
		void SetViewMatrix(const tMatrix &m)
		{
			// GLenum preMode = _mMode;
			// _mMode = GL_PROJECTION;
			_viewMat[sizeView()] = m;
			// _mMode = preMode;
		}
		/*
		void SetViewMatrix(const glm::vec3 eye, const glm::vec3 ref, const glm::vec3 up)
		{ 	/// Don't remember if this is ever used...
			SetViewMatrix(glm::lookAt(eye, ref, up));
		}
		 */
		void Mult(const tMatrix &m)
		{
			if (size() < 0) return;
			if (_mMode == MODELVIEW_MAT)
				_items[size()] = _items[size()] * m;
			else
				_viewMat[sizeView()] = _viewMat[sizeView()] * m;
		}
		void PreMult(const tMatrix &m)
		{
			if (size() < 0) return;
			if (_mMode == MODELVIEW_MAT)
				_items[size()] = m * _items[size()];
			else
				_viewMat[sizeView()] = m * _viewMat[sizeView()];
		}
		void Push(void) {
			if (Full()){
				std::cout << "Stack Full!" << std::endl;
				return;
			}

			if (_mMode == MODELVIEW_MAT)
			{
				// _items[size() + 1] = _items[size()];
				// _top = _top + 1;
				_items.push_back(_items.back());
			}
			else
			{
				// _viewMat[sizeView() + 1] = _viewMat[sizeView()];
				// _topView = _topView + 1;
				_viewMat.push_back(_viewMat.back());
			}
		}
		void Pop(void){
			if (Empty()){
				std::cout << "Stack Empty!" << std::endl;
				return;
			}
			if (_mMode == MODELVIEW_MAT)
				_items.pop_back();
				// _top = _top - 1;
			else
				_viewMat.pop_back();
				// _topView = _topView - 1;
		}

		MAT_STACK_MODE getMatrixMode() const
		{
			return _mMode;
		}

		/*
		void print(void) {
			std::cout << "View Matrix:\m" ;
			for( int i = 0 ; i < 4; i++) {
				for( int j = 0 ; j < 4; j++) {
					std::cout << View()[i][j] << " " ;
				}
				std::cout << "\n" ;
			}
			std::cout << "ModelView Matrix:\n" ;
			for( int i = 0 ; i < 4; i++) {
				for( int j = 0 ; j < 4; j++) {
					std::cout << Top()[i][j] << " " ;
				}
				std::cout << "\n" ;
			}
		}
		*/
	};

}

#endif

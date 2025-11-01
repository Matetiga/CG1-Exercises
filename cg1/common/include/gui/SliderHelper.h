#pragma once

#include <nanogui/slider.h>
#include <nanogui/textbox.h>

namespace nse
{
	namespace gui
	{
		nanogui::Slider* AddWidgetWithSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, nanogui::Widget*& outWidget);

		nanogui::Slider* AddLabeledSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue);
		 
		nanogui::Slider* AddLabeledSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, nanogui::TextBox*& out_label);

		nanogui::Slider* AddLabeledSliderWithDefaultDisplay(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, std::streamsize displayPrecision);

		class VectorInput
		{
		public:
			VectorInput(nanogui::Widget* parent, const std::string& prefix, const Eigen::Vector3f& min, const Eigen::Vector3f& max, const Eigen::Vector3f& current, std::function<void(const Eigen::Vector3f&)> callback);

			Eigen::Vector3f Value() const;
			
			void SetBounds(const Eigen::Vector3f& lower, const Eigen::Vector3f& upper);
			void SetValue(const Eigen::Vector3f& value);

		private:
			nanogui::Slider *sldX, *sldY, *sldZ;
		};
	}
}
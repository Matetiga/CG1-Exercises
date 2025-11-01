#include <gui/SliderHelper.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>

using namespace nse::gui;

nanogui::Slider* nse::gui::AddWidgetWithSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, nanogui::Widget*& outWidget)
{
	outWidget = new nanogui::Widget(parent);
	outWidget->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 6));
	new nanogui::Label(outWidget, caption);

	auto slider = new nanogui::Slider(outWidget);
	slider->setFixedWidth(100);
	slider->setValue(defaultValue);
	slider->setRange(range);

	return slider;
}

nanogui::Slider* nse::gui::AddLabeledSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue) 
{ 
	nanogui::Widget* widget; 
	return AddWidgetWithSlider(parent, caption, range, defaultValue, widget); 
} 

 
nanogui::Slider* nse::gui::AddLabeledSlider(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, nanogui::TextBox*& out_label) 
{ 
	nanogui::Widget* widget; 
	auto slider = AddWidgetWithSlider(parent, caption, range, defaultValue, widget);	 
 
	out_label = new nanogui::TextBox(widget); 
	out_label->setFixedSize(Eigen::Vector2i(60, 25)); 
 
	return slider; 
} 

nanogui::Slider* nse::gui::AddLabeledSliderWithDefaultDisplay(nanogui::Widget* parent, const std::string& caption, const std::pair<float, float>& range, float defaultValue, std::streamsize displayPrecision)
{
	nanogui::TextBox* txt;
	auto sld = AddLabeledSlider(parent, caption, range, defaultValue, txt);
	sld->setCallback([txt, displayPrecision](float value) {
		std::stringstream ss;
		ss.precision(displayPrecision);
		ss << std::fixed << value;
		txt->setValue(ss.str());
	});

	sld->callback()(sld->value());

	return sld;
}

VectorInput::VectorInput(nanogui::Widget* parent, const std::string& prefix, const Eigen::Vector3f& min, const Eigen::Vector3f& max, const Eigen::Vector3f& current, std::function<void(const Eigen::Vector3f&)> callback)
{
	sldX = nse::gui::AddLabeledSliderWithDefaultDisplay(parent, prefix + " X", std::make_pair(min.x(), max.x()), current.x(), 2);
	sldY = nse::gui::AddLabeledSliderWithDefaultDisplay(parent, prefix + " Y", std::make_pair(min.y(), max.y()), current.y(), 2);
	sldZ = nse::gui::AddLabeledSliderWithDefaultDisplay(parent, prefix + " Z", std::make_pair(min.z(), max.z()), current.z(), 2);
	auto oldXCb = sldX->callback();
	auto oldYCb = sldY->callback();
	auto oldZCb = sldZ->callback();
	auto update = [callback, this]() {
		auto value = Value();
		callback(value);
	};
	sldX->setCallback([oldXCb, update](float value) {oldXCb(value); update(); });
	sldY->setCallback([oldYCb, update](float value) {oldYCb(value); update(); });
	sldZ->setCallback([oldZCb, update](float value) {oldZCb(value); update(); });
}

Eigen::Vector3f VectorInput::Value() const
{
	return Eigen::Vector3f(sldX->value(), sldY->value(), sldZ->value());
}

void VectorInput::SetBounds(const Eigen::Vector3f& lower, const Eigen::Vector3f& upper)
{
	sldX->setRange(std::make_pair(lower.x(), upper.x()));
	sldY->setRange(std::make_pair(lower.y(), upper.y()));
	sldZ->setRange(std::make_pair(lower.z(), upper.z()));
}

void VectorInput::SetValue(const Eigen::Vector3f& value)
{
	sldX->setValue(value.x()); sldX->callback()(sldX->value());
	sldY->setValue(value.y()); sldY->callback()(sldY->value());
	sldZ->setValue(value.z()); sldZ->callback()(sldZ->value());
}
#include "shipcon2_gui/standard/single_engine.hh"

namespace shipcon::gui::standard
{
  SingleEnginePanel::SingleEnginePanel( QWidget *parent ):
  rviz_common::Panel( parent )
  {
    show();
    parentWidget()->show();
  }

  SingleEnginePanel::~SingleEnginePanel()
  {
    delete svg_panel_;
  }

  void SingleEnginePanel::onInitialize()
  {
    svg_panel_ = new QSvgRenderer( QString::fromStdString( PANEL_SVG_PATH ) );
  }

  void SingleEnginePanel::load(const rviz_common::Config &config)
  {
      rviz_common::Panel::load(config);
  }

  void SingleEnginePanel::save(rviz_common::Config config) const
  {
      rviz_common::Panel::save(config);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( shipcon::gui::standard::SingleEnginePanel, rviz_common::Panel )
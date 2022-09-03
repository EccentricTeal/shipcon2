#ifndef SHIPCON2_GUI__STANDARD__SINGLE_ENGINE__HH
#define SHIPCON2_GUI__STANDARD__SINGLE_ENGINE__HH

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QtWidgets>
#include <qt5/QtSvg/QSvgRenderer>

#include <string>
#endif

namespace shipcon::gui::standard
{
  class SingleEnginePanel : public rviz_common::Panel
  {
    Q_OBJECT
    
    /* Constants */
    private:
      const std::string PANEL_SVG_PATH = "img/single_engine.svg";

    /* Member */
    private:
      double scale_rate;
      QSvgRenderer* svg_panel_;

    /* Constructor, Destructro */
    public:
      SingleEnginePanel( QWidget *parent = nullptr );
      ~SingleEnginePanel();

    /* Method */
    public:
      virtual void onInitialize();
      virtual void load( const rviz_common::Config &config );
      virtual void save( rviz_common::Config config ) const;

  };
}










#endif
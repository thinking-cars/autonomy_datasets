// Copyright Thinking Cars GmbH
// SPDX-License-Identifier: Apache-2.0

#ifndef PLAYBACK_PANEL_HPP_
#define PLAYBACK_PANEL_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <QEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QTimer>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

#include "autonomy_datasets_msgs/srv/request_samples.hpp"

/**
 * \class Playback Panel
 * \brief Steps through a dataset published by autonomy_datasets.
 *
 */
namespace autonomy_datasets_rviz_plugins {

class PlaybackPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  /**
   * @brief Creates the panel and its widgets, without connecting to a service yet.
   *
   * @param parent widget taking ownership of the panel, as usual for Qt widgets
   */
  explicit PlaybackPanel(QWidget* parent = nullptr);

  /**
   * @brief Destroys the panel, dropping the service client before the RViz node goes away.
   */
  ~PlaybackPanel() override;

  /**
   * @brief Copy construction is disabled because the panel owns Qt and ROS resources.
   */
  PlaybackPanel(const PlaybackPanel&) = delete;

  /**
   * @brief Copy assignment is disabled because the panel owns Qt and ROS resources.
   */
  PlaybackPanel& operator=(const PlaybackPanel&) = delete;

  /**
   * @brief Move construction is disabled because RViz owns the panel lifecycle.
   */
  PlaybackPanel(PlaybackPanel&&) = delete;

  /**
   * @brief Move assignment is disabled because RViz owns the panel lifecycle.
   */
  PlaybackPanel& operator=(PlaybackPanel&&) = delete;

  /**
   * @brief Picks up the ROS node of RViz and creates the client for the configured service.
   */
  void onInitialize() override;

  /**
   * @brief Restores the service name and step size stored in the RViz configuration.
   *
   * @param config RViz configuration to read the panel settings from
   */
  void load(const rviz_common::Config& config) override;

  /**
   * @brief Stores the service name and step size in the RViz configuration.
   *
   * @param config RViz configuration to write the panel settings to
   */
  void save(rviz_common::Config config) const override;

 Q_SIGNALS:
  /**
   * @brief Reports the outcome of a sample request from the thread that received the response.
   *
   * Connected to applyRequestResult() as a queued connection, so that the widgets are only ever
   * touched by the GUI thread, no matter which thread the executor of RViz answers the request on.
   * Qt drops the queued call if the panel is closed before it is delivered.
   *
   * @param position description of the sample the dataset node published last
   * @param status description of the outcome, as reported by the service
   * @param success whether all requested samples have been published
   */
  // Moc generates the definition of the signal with parameter names of its own making
  // NOLINTNEXTLINE(readability-inconsistent-declaration-parameter-name)
  void requestFinished(const QString& position, const QString& status, bool success);

 private:  // NOLINT(readability-redundant-access-specifiers)
  /**
   * @brief Re-creates the service client after the service name has been edited.
   */
  void onServiceNameChanged();

  /**
   * @brief Requests no sample at all, which stops the playback at the current sample.
   */
  void onPauseClicked();

  /**
   * @brief Requests the next samples, which advances the playback by the configured step size.
   */
  void onStepClicked();

  /**
   * @brief Requests the entered sample IDs, which skips the playback ahead to them.
   */
  void onSkipClicked();

  /**
   * @brief Requests all remaining samples, which runs the playback to the end of the dataset.
   */
  void onPlayClicked();

  /**
   * @brief Tracks whether the service is available and reflects it in the panel.
   */
  void updateAvailability();

  /**
   * @brief Shows the outcome of a sample request and re-enables the controls.
   *
   * Only ever called on the GUI thread, as the response of a request may reach the panel on
   * whichever thread the executor of RViz runs on.
   *
   * @param position description of the sample the dataset node published last
   * @param status description of the outcome, as reported by the service
   * @param success whether all requested samples have been published
   */
  void applyRequestResult(const QString& position, const QString& status, bool success);

  /**
   * @brief Re-shortens the reported texts whenever one of the labels showing them is resized.
   *
   * @param watched object the event is delivered to
   * @param event event delivered to the watched object
   * @return whether the event has been handled and is not to be delivered any further
   */
  bool eventFilter(QObject* watched, QEvent* event) override;

  /**
   * @brief Creates the widgets of the panel and connects them to their handlers.
   */
  void setupUi();

  /**
   * @brief Reports which sample the dataset node published last.
   *
   * @param text description of the sample
   */
  void showPosition(const QString& text);

  /**
   * @brief Reports what the panel is waiting for or what a request resulted in.
   *
   * @param text description of the state or of the outcome
   */
  void showStatus(const QString& text);

  /**
   * @brief Shows as much of a text as fits into a label and offers all of it as its tooltip.
   *
   * The panel is docked beside the render window of RViz, where a scene ID or a message of the
   * service would widen it far beyond the width its controls need.
   *
   * @param label label to show the text in
   * @param text text to show, shortened at its end where it does not fit
   */
  static void showElided(QLabel* label, const QString& text);

  /**
   * @brief Creates the service client for the service name currently entered in the panel.
   */
  void createClient();

  /**
   * @brief Sends a sample request and reports its response through requestFinished().
   *
   * @param mode requested playback mode, one of the MODE_* constants of the service
   * @param num_samples number of samples to publish in MODE_NEXT_SAMPLES
   * @param sample_ids IDs of the samples to publish in MODE_SAMPLE_IDS
   * @param pending_status description of the request, shown while it is being processed
   */
  void sendRequest(uint8_t mode, uint64_t num_samples, const std::vector<uint64_t>& sample_ids, const QString& pending_status);

  /**
   * @brief Enables the controls that may be used while no request is being processed.
   */
  void updateControlState();

  /**
   * @brief Reads the sample IDs entered in the panel.
   *
   * @param error set to a description of the first malformed entry, if there is one
   * @return sample IDs in the order they were entered, empty if an entry is malformed
   */
  std::vector<uint64_t> parseSampleIds(QString* error) const;

  /**
   * @brief Describes the sample a response reports as published last.
   *
   * @param response response of the sample request
   * @return description of the last published sample, or a placeholder if none was published
   */
  static QString describePosition(const autonomy_datasets_msgs::srv::RequestSamples::Response& response);

  //! Name of the request_samples service of the dataset node to control
  QLineEdit* service_name_edit_{nullptr};
  //! Number of samples a step advances the playback by
  QSpinBox* step_size_spin_{nullptr};
  //! Sample IDs to skip ahead to
  QLineEdit* sample_ids_edit_{nullptr};
  //! Stops the playback at the current sample
  QPushButton* pause_button_{nullptr};
  //! Advances the playback by the configured step size
  QPushButton* step_button_{nullptr};
  //! Skips the playback ahead to the entered sample IDs
  QPushButton* skip_button_{nullptr};
  //! Runs the playback to the end of the dataset
  QPushButton* play_button_{nullptr};
  //! Sample the dataset node published last
  QLabel* position_label_{nullptr};
  //! Outcome of the last request, or why no request can be sent
  QLabel* status_label_{nullptr};
  //! Polls whether the service is available, as the panel cannot be notified about it
  QTimer* availability_timer_{nullptr};

  //! ROS node of RViz, which its executor spins
  rclcpp::Node::SharedPtr node_;
  //! Client of the request_samples service of the dataset node
  rclcpp::Client<autonomy_datasets_msgs::srv::RequestSamples>::SharedPtr client_;
  //! Whether a request has been sent whose response has not arrived yet
  bool request_in_flight_{false};
  //! Whether the service was available the last time it was polled
  bool service_available_{false};
  //! Full text reported by position_label_, of which only what fits is shown
  QString position_text_;
  //! Full text reported by status_label_, of which only what fits is shown
  QString status_text_;
};

}  // namespace autonomy_datasets_rviz_plugins

#endif  // PLAYBACK_PANEL_HPP_

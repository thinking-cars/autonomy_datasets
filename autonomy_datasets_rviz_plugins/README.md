# `autonomy_datasets_rviz_plugins`

RViz plugins to control the playback of the datasets published by autonomy_datasets

### Playback panel

Steps and skips through a dataset from within RViz, by calling the `request_samples` service of
the [`autonomy_datasets`](../autonomy_datasets/README.md) node. It is part of the RViz
configuration the dataset launch file starts, and can be added to any other one via
_Panels → Add New Panel → autonomy_datasets_rviz_plugins → Playback_.

| Control | Effect |
| --- | --- |
| **Service** | Name of the `request_samples` service to control, for a dataset node that is not started under its default name |
| **Pause** | Stops the playback at the current sample. The first request of any kind takes control of the playback, so samples are published only on request from then on |
| **Play to End** | Publishes all remaining samples. The dataset node answers this request only once the dataset ends, so the playback cannot be paused again until then |
| **Step** | Publishes the next _n_ samples, whichever they are |
| **Skip To** | Publishes the samples with the entered IDs, skipping every sample in between |

Samples are identified by their position within the playback pass, starting at 0. Playback is
forward-only, so a sample that has already been passed cannot be skipped back to; the panel
reports it as not published.

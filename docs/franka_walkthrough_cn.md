# Franka 机械臂训练教程

我们演示了如何在真实机械臂上使用 HIL-SERL，包含论文中提出的4个任务：RAM 插入、USB 拾取与插入、物体交接和翻鸡蛋。这些代表性任务的选择是为了突出我们代码库支持的各种应用场景，如双臂支持（物体交接）、多阶段奖励任务（USB 拾取与插入）和动态任务（翻鸡蛋）。我们提供了 RAM 插入任务的整个训练和评估流程的详细说明和技巧，因此建议仔细阅读作为入门起点。

## 1. RAM 插入

### 流程

#### 机器人设置

如果尚未完成，请先阅读 [README.md](../README.md) 中关于设置 Python 环境和使用我们的 Franka 控制器的说明。以下步骤假设所有安装步骤都已完成且 Python 环境已激活。关于工作空间设置，请参阅论文中的工作空间设置图像。

1. 通过编辑 `Desk > Settings > End-effector > Mechanical Data > Mass` 来调整腕部相机的重量。
2. 在 Franka Desk 中解锁机器人并激活 FCI。`franka_server` 启动文件位于 [serl\_robot\_infra/robot\_servers/launch\_right\_server.sh](../serl_robot_infra/robot_servers/launch_right_server.sh)。您需要编辑 `setup.bash` 路径以及 `python franka_server.py` 命令的标志位。关于如何设置这些标志位，请参阅 [serl\_robot\_infra/README.md](../serl_robot_infra/README.md)。要启动服务器，请运行：

```bash
bash serl_robot_infra/robot_servers/launch_right_server.sh
```

#### 编辑训练配置

对于每个任务，我们在 experiments 文件夹中创建一个文件夹，用于存储数据（即任务演示、奖励分类器数据、训练运行检查点）、启动脚本和训练配置（参见 [experiments/ram\_insertion](../examples/experiments/ram_insertion/)）。接下来，我们将逐步介绍如何修改训练配置 [experiments/ram\_insertion/config.py](../examples/experiments/ram_insertion/config.py)) 以开始训练：

1. 首先，在 `EnvConfig` 类中，将 `SERVER_URL` 更改为您正在运行的 Franka 服务器的 URL。
2. 接下来，我们需要配置相机。对于这个任务，我们使用两个腕部相机。任务中使用的所有相机（奖励分类器和策略训练）都列在 `REALSENSE_CAMERAS` 中，相应的图像裁剪设置在 `EnvConfig` 类的 `IMAGE_CROP` 中。策略训练和奖励分类器使用的相机键分别列在 `TrainConfig` 类的 `image_keys` 和 `classifier_keys` 中。将 `REALSENSE_CAMERAS` 中的序列号更改为您设置中相机的序列号（可以在 RealSense Viewer 应用程序中找到）。要调整图像裁剪（以及可能的曝光），您可以运行奖励分类器数据收集脚本（参见步骤6）或演示数据收集脚本（参见步骤8）来可视化相机输入。
3. 最后，我们需要为训练过程收集一些姿态。对于这个任务，`TARGET_POSE` 是指 RAM 条完全插入主板时的机械臂姿态，`GRASP_POSE` 是指抓取放在支架上的 RAM 条时的机械臂姿态，`RESET_POSE` 是指重置到的机械臂姿态。`ABS_POSE_LIMIT_HIGH` 和 `ABS_POSE_LIMIT_LOW` 决定了策略的边界框。我们启用了 `RANDOM_RESET`，这意味着每次重置时会在 `RESET_POSE` 周围进行随机化（`RANDOM_XY_RANGE` 和 `RANDOM_RZ_RANGE` 控制随机化的程度）。您应该重新收集 `TARGET_POSE`、`GRASP_POSE`，并确保为安全探索设置了边界框。要收集 Franka 臂的当前姿态，请运行：
   ```bash
   curl -X POST http://<FRANKA_SERVER_URL>:5000/getpos_euler
   ```

#### 训练奖励分类器

此任务的奖励通过在相机图像上训练的奖励分类器给出。对于此任务，我们使用与训练策略相同的两个腕部图像来训练奖励分类器。以下步骤介绍收集分类器数据和训练奖励分类器。

> **提示**：根据区分任务奖励的难度，有时使用单独的相机或同一相机图像的多个放大裁剪可能会有帮助。

1. 首先，我们需要为分类器收集训练数据。导航到 examples 文件夹并运行：
   ```bash
   cd examples
   python record_success_fail.py --exp_name ram_insertion --successes_needed 200
   ```
   当脚本运行时，默认情况下所有记录的转换都标记为负（或无奖励）。如果在转换过程中按住空格键，该转换将标记为正。当收集到足够多的正向转换时脚本将终止（默认为 200，但可以通过 successes_needed 标志设置）。对于此任务，您应该收集 RAM 条放在工作空间各个位置的负向转换以及插入过程中的转换，并在 RAM 完全插入时按下空格键。分类器数据将保存到文件夹 `experiments/ram_insertion/classifier_data`。
   > **提示**：为了训练一个能够抵抗误报（这对于训练成功的策略非常重要），我们发现收集 2-3 倍于正向转换的负向转换会有帮助，以覆盖所有失败模式。例如，对于 RAM 插入，这可能包括在主板上的错误位置尝试插入、半插入，或者将 RAM 条放在插槽旁边。
2. 要训练奖励分类器，请导航到此任务的实验文件夹并运行：
   ```bash
   cd experiments/ram_insertion
   python ../../train_reward_classifier.py --exp_name ram_insertion
   ```
   奖励分类器将在训练配置中分类器键指定的相机图像上进行训练。训练好的分类器将保存到文件夹 `experiments/ram_insertion/classifier_ckpt`。

#### 录制演示

少量人类演示对于加速强化学习过程至关重要，对于此任务，我们使用 20 个演示。

1. 要使用 spacemouse 录制 20 个演示，请运行：
   ```bash
   python ../../record_demos.py --exp_name ram_insertion --successes_needed 20
   ```
   一旦奖励分类器认为 episode 成功或 episode 超时，机器人将重置。当收集到 20 个成功演示后脚本将终止，这些演示将保存到文件夹 `experiments/ram_insertion/demo_data`。
   > **提示**：在演示数据收集过程中，您可能会注意到奖励分类器输出误报（在没有成功插入的情况下 episode 终止并给出奖励）或漏报（尽管成功插入但未给出奖励）。在这种情况下，您应该收集额外的分类器数据来针对观察到的分类器失败模式（例如，如果分类器对在空中拿着 RAM 条给出误报，您应该收集更多此类情况的负向数据）。或者，您也可以调整奖励分类器的阈值，尽管我们强烈建议先收集额外的分类器数据（或者如果需要的话添加更多分类器相机/图像）。
   > 提示：为了训练一个能够抵抗误报的分类器（这对于训练成功的策略非常重要），我们发现收集 2-3 倍于正向转换的负向转换会有帮助，以覆盖所有失败模式。例如，对于 RAM 插入，这可能包括在主板上的错误位置尝试插入、半插入，或者将 RAM 条放在插槽旁边。

#### 策略训练

策略训练通过 actor 节点异步完成，该节点负责在环境中 rollout 策略并将收集到的转换发送给 learner，以及负责训练策略并将更新后的策略发送回 actor 的 learner 节点。策略训练期间，actor 和 learner 都应该运行。

1. 在 RAM 插入实验对应的文件夹（[experiments/ram\_insertion](../examples/experiments/ram_insertion/)）中，您会找到 `run_actor.sh` 和 `run_learner.sh`。在这两个脚本中，编辑 `checkpoint_path` 指向保存检查点和训练过程中生成的其他数据的文件夹，在 `run_learner.sh` 中，编辑 `demo_path` 指向录制的演示的路径（如果有多个演示文件，您可以提供多个 `demo_path` 标志）。要开始训练，启动两个节点：
   ```bash
   bash run_actor.sh
   bash run_learner.sh
   ```
   > **提示**：要恢复之前的训练运行，只需将 `checkpoint_path` 编辑为指向前一次运行的文件夹，代码将自动加载最新的检查点和训练缓冲区数据并恢复训练。
2. 在训练期间，您应该根据需要使用 spacemouse 进行一些干预以加快训练运行，尤其是在训练开始时或当策略重复探索不正确行为时（即，将 RAM 条移离主板）。作为参考，在开启随机化并给予偶尔干预的情况下，策略大约需要 1.5 小时才能收敛到 100% 成功率。
   > **提示**：对于这个任务，我们在抓取姿态上添加了一些额外的随机化。因此，您应该定期重新抓取 RAM 条并按 F1 将 RAM 放回支架。
3. 要评估训练好的策略，请在 `run_actor.sh` 中添加标志 `--eval_checkpoint_step=要评估的检查点编号` 和 `--eval_n_trajs=评估次数`。然后启动 actor：
   ```bash
   bash run_actor.sh
   ```

## 2. USB 拾取与插入

### 流程

#### 机器人设置

这些步骤假设您已完成所有安装程序且 Python 环境已激活。关于工作空间设置，请参阅论文中的工作空间设置图像。

1. 通过编辑 `Desk > Settings > End-effector > Mechanical Data > Mass` 来调整腕部相机的重量。
2. 在 Franka Desk 中解锁机器人并激活 FCI。如果尚未完成，您需要编辑 `franka_server` 启动文件（请参阅 RAM 插入说明中的步骤 2）。要启动服务器，请运行：

```bash
bash serl_robot_infra/robot_servers/launch_right_server.sh
```

#### 编辑训练配置

接下来，我们需要编辑训练配置 [experiments/usb\_pickup\_insertion/config.py](../examples/experiments/usb_pickup_insertion/config.py)) 以开始训练：

1. 首先，在 `EnvConfig` 类中，将 `SERVER_URL` 更改为您正在运行的 Franka 服务器的 URL。
2. 接下来，我们需要配置相机。对于此任务，我们使用了 2 个腕部相机和 1 个侧面相机（策略和分类器使用不同的裁剪）。将 `REALSENSE_CAMERAS` 中的序列号更改为您设置中相机的序列号（可以在 RealSense Viewer 应用程序中找到），并在 `IMAGE_CROP` 中调整图像裁剪。要可视化相机视图，您可以运行奖励分类器数据收集脚本或演示数据收集脚本。

> **注意**：此配置是如何使用同一相机的多个裁剪视图的示例。对于同一相机的每个裁剪视图，您需要在 `REALSENSE_CAMERAS` 和 `IMAGE_CROP` 中添加一个条目。请注意，我们在 `USBEnv` 的 `init_cameras` 函数中避免多次初始化同一相机（参见 [experiments/usb\_pickup\_insertion/wrapper.py](../examples/experiments/usb_pickup_insertion/wrapper.py)）

1. 最后，您需要收集 `TARGET_POSE`，对于此任务，这是指 USB 完全插入端口时抓取 USB 的机械臂姿态。同时，确保为安全探索设置了边界框（参见 `ABS_POSE_LIMIT_HIGH` 和 `ABS_POSE_LIMIT_LOW`）。请注意，`RESET_POSE`（episode 重置时机械臂放置 USB 前移动到的位置）已定义，且 `RANDOM_RESET` 已启用。要收集 Franka 臂的当前姿态，请运行：
   ```bash
   curl -X POST http://<FRANKA_SERVER_URL>:5000/getpos_euler
   ```

#### 训练奖励分类器

对于此任务，我们使用侧面相机的裁剪视图来训练奖励分类器。以下步骤介绍收集分类器数据和训练奖励分类器。

1. 首先，我们需要为分类器收集训练数据。导航到 examples 文件夹并运行：
   ```bash
   cd examples
   python record_success_fail.py --exp_name usb_pickup_insertion
   ```
   有关使用此脚本的更多详细信息，请参阅 RAM 插入说明中的步骤 6。对于此任务，我们发现收集 RAM 条放在工作空间各个位置的负向转换、部分成功的插入以及成功插入到错误 USB 端口的转换会很有帮助。分类器数据将保存到文件夹 `experiments/usb_pickup_insertion/classifier_data`。
2. 要训练奖励分类器，请导航到此任务的实验文件夹并运行：
   ```bash
   cd experiments/usb_pickup_insertion
   python ../../train_reward_classifier.py --exp_name usb_pickup_insertion
   ```
   奖励分类器将在训练配置中分类器键指定的相机图像上进行训练。训练好的分类器将保存到文件夹 `experiments/usb_pickup_insertion/classifier_ckpt`。

#### 录制演示

少量人类演示对于加速强化学习过程至关重要，对于此任务，我们使用 20 个演示。

1. 要使用 spacemouse 录制 20 个演示，请运行：
   ```bash
   python ../../record_demos.py --exp_name usb_pickup_insertion --successes_needed 20
   ```
   一旦奖励分类器认为 episode 成功或 episode 超时，机器人将重置。这也是验证奖励分类器和重置是否按预期工作的好机会。当收集到 20 个成功演示后脚本将终止，这些演示将保存到文件夹 `experiments/usb_pickup_insertion/demo_data`。

#### 策略训练

1. 在 USB 拾取与插入实验对应的文件夹（[experiments/usb\_pickup\_insertion](../examples/experiments/usb_pickup_insertion/)）中，您会找到 `run_actor.sh` 和 `run_learner.sh`。在这两个脚本中，编辑 `checkpoint_path` 指向保存检查点和训练过程中生成的其他数据的文件夹，在 `run_learner.sh` 中，编辑 `demo_path` 指向录制的演示的路径（如果有多个演示文件，您可以提供多个 `demo_path` 标志）。要开始训练，启动两个节点：
   ```bash
   bash run_actor.sh
   bash run_learner.sh
   ```
2. 在训练期间，您应该根据需要使用 spacemouse 进行一些干预以加快训练运行，尤其是在训练开始时或当策略重复探索不正确行为时（即，将 USB 移离主板）。作为参考，我们的策略大约需要 2.5 小时才能收敛到 100% 成功率。
3. 要评估训练好的策略，请在 `run_actor.sh` 中添加标志 `--eval_checkpoint_step=要评估的检查点编号` 和 `--eval_n_trajs=评估次数`。然后启动 actor：
   ```bash
   bash run_actor.sh
   ```

## 3. 物体交接

说明即将推出！

## 4. 翻鸡蛋

说明即将推出！

## 成功技巧补充

- 关于人类干预的技巧
  - 首先最重要的是，使用 spacemouse 进行干预（键盘之类的东西精度低得多）！
  - 在训练初期更频繁地进行干预（频率可以是每隔一个 episode 或每个 episode 干预一些时间步）。重要的是在让策略探索（强化学习必需的）和通过干预引导策略高效探索之间取得平衡。例如，对于插入任务（即 RAM 插入），在训练初期，策略会有很多随机运动。我们通常会让它探索这些随机运动 20-30 个时间步，然后干预引导物体接近插入端口，在那里让策略练习插入。在探索和干预之间交替进行，可以让策略有机会探索任务的每个部分，同时也不会浪费时间探索完全不正确的行为（即在远离插入端口的边界框边缘探索随机运动）。我们还发现，即使在训练初期，帮助策略完成任务并获得奖励半频繁也会有好处（即，让 1/3 或更多的 episode 获得奖励）——频繁的奖励有助于价值备份更快传播，加速训练。
  - 随着策略开始表现得更合理（策略可以偶尔自己成功完成任务，几乎不需要/不需要干预），我们可以显著减少干预频率。在这个阶段，我们通常会抑制干预，除非策略反复犯同样的错误。
  - 有时，我们可能希望训练好的策略具有更 robust 的重试行为（意味着即使早期犯了错误，策略也能成功完成任务）或对外界干扰更 robust。在这种情况下，我们也可以使用干预来帮助它练习这些边缘情况。例如，如果我们希望 USB 拾取和插入策略对某种边缘情况 robust（即第一次尝试失败后将 USB 掉在主板附近，仍能成功插入 USB），我们可以干预使策略犯这个错误，并让它练习从中恢复。使用干预将策略带到可以练习这些恢复行为的地方，对于达到 100% 成功率是有效的，因为这些错误可能很少发生，但策略仍需要学习如何从中恢复，才能从 97% 成功率的策略提高到 100% 成功率的策略。

更多内容即将推出！

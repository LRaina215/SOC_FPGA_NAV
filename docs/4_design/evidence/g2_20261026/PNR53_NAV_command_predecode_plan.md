导航共同长链的入口是 MATCH_CMD 0x700C，不只是 POSE_COMMIT 0x2070。PNR53 中 a_q[7] 发起了2546个负终点；最差到XT帧RAM CEB为29.880ns、15级，布线占79.017%。实际链经MATCH译码、wd_q[0]、frame_can_start、RAW受理和support/map ready反压，再到XT/高程/提取使能。

建议下一候选只改 nav_subsystem：在原 B_IDLE 最后一个普通RF写分支，与 a_q/wd_q/rf_wr 同沿捕获六个独立命令脉冲：MATCH的EVAL/UPDATE/CLEAR、POSE_COMMIT、MAP的SWEEP/CLEAR。每拍默认清零。地址先按原方式丢低两位，命令位来自同次 bus_wdata；多位同时置1仍合法，由原消费端保持优先级。替换对应消费连接，不改原配置寄存器或采样时刻，也不改support的raw_frame_accept帧起点。

E0受理后，新脉冲与原组合译码同时可见；E1消费者动作和RF ACK保持原拍；E2原B_END气泡保持。不能把译码结果再打一拍，也不能把原始bus_req提前直连frame_can_start——后者会比原来早一拍阻止RAW。被阻塞的窗口/MBX读不能留下命令脉冲，req在ACK后一直保持时也不能偷偷新增“只执行一次”规则。

旧pose预译码可复用同沿捕获思路和参考脉冲夹具。旧541次总线事务/83次POSE证明不覆盖当前MATCH/MAP、ownership、保留帧和WAIT_POINTS；旧生成器还会编辑活repo，且匹配的旧support连接已不存在，不能原样运行。

物理闭合仍未知。仅从报告减掉前三层译码及其布线5.505ns，余下仍约24.375ns；这只是算术说明，不能当新时序预测。反压链尚在，新预译码D端也要重新查上游总线/仲裁路径。若还需注册ready或加缓冲，属于另一份流握手合同，不能混入此最小方案。

未来验证重点：全地址/数据的逐拍脉冲等式与独立X/Z复位检查；双主仲裁、长stall、B_END和ACK不变；console已确认posted字节、TXshadow E0读/B_RF写、raw注入GO/长度/覆盖不变；RAW受理与命令同沿、SOF、末点写排空、HOLD/clear/0点/第二遍保留帧；MAP双命令及SAFE/IR/lease停止交界。最后核真实C映射中的FF/CE和完整物理报告，不能用源属性或旧证明代替。

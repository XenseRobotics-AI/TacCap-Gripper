"""UI-side timing and real transfer progress; never estimates whole-job percent."""

import time


class Activity:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.active = False
        self.stage = ""
        self.started = self.changed = self.last_progress = self.last_log = clock()
        self.transfer = None

    def begin(self, stage):
        self.active = True
        self.started = self.clock()
        self.set_stage(stage)

    def set_stage(self, stage):
        self.stage = stage
        self.changed = self.last_progress = self.last_log = self.clock()
        self.transfer = None

    def update(self, done, total, unit):
        self.transfer = (max(0, done), max(0, total), unit)
        self.last_progress = self.clock()

    def detail(self):
        now = self.clock()
        elapsed = max(0, int(now - self.started))
        stage_time = max(0, int(now - self.changed))
        detail = (
            f"已用时 {elapsed // 60:02d}:{elapsed % 60:02d} · 当前步骤 {stage_time} 秒"
        )
        if self.transfer:
            done, total, unit = self.transfer
            percent = min(100, int(100 * done / total)) if total else 0
            detail += f" · 传输 {percent}%（{done:,}/{total:,} {unit}）"
            if total and done >= total:
                detail += " · 传输结束，仍需完成校验"
            elif now - self.last_progress >= 5:
                detail += " · 暂无新传输回调，等待设备回执"
        return detail

    def heartbeat(self):
        now = self.clock()
        if not self.active or now - self.last_log < 5:
            return None
        self.last_log = now
        return f"进行中：{self.stage}；{self.detail()}"

    def finish(self):
        self.active = False

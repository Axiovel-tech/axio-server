from types import SimpleNamespace

from flockwave.server.ext.mavlink.extension import MAVLinkDronesExtension


async def test_show_sync_removal_broadcasts_tombstone():
    broadcasts = []

    class Hub:
        @staticmethod
        def create_notification(body):
            return body

        async def broadcast_message(self, message):
            broadcasts.append(message)

    extension = MAVLinkDronesExtension()
    extension._app = SimpleNamespace(message_hub=Hub())
    pending = []
    extension.run_in_background = pending.append
    extension._show_sync_status["1"] = {"source": "uwb-ltc", "committed": True}
    extension._show_sync_status_updated_at["1"] = 1.0

    extension._remove_show_sync_status("1")
    await pending.pop()()

    assert "1" not in extension._show_sync_status
    assert "1" not in extension._show_sync_status_updated_at
    assert broadcasts == [{"type": "X-SHOW-SYNC", "status": {"1": None}}]

from email.message import Message
import io
import json
import urllib.error
import urllib.request
import urllib.response

import pytest

from scripts import launch


def fake_transport(monkeypatch, responder):
    requests = []
    class Transport(urllib.request.HTTPHandler):
        def http_open(self, request):
            requests.append(request)
            code, headers, body = responder(request)
            response = urllib.response.addinfourl(io.BytesIO(body), headers, request.full_url, code)
            response.msg = "OK" if code == 200 else "Redirect"
            return response
    original = urllib.request.build_opener
    monkeypatch.setattr(launch.urllib.request, "build_opener", lambda *handlers: original(*handlers, Transport()))
    return requests


@pytest.mark.parametrize("code", [301, 302, 303, 307, 308])
@pytest.mark.parametrize("destination", ["http://example.invalid/capture", "http://127.0.0.1:8010/other"])
def test_launcher_never_forwards_authorization_through_redirect(monkeypatch, code, destination):
    headers = Message()
    headers["Location"] = destination
    requests = fake_transport(monkeypatch, lambda request: (code, headers, b""))
    with pytest.raises(urllib.error.HTTPError) as caught:
        launch.request_json("http://127.0.0.1:8010", "/api/state", "placeholder-launch-token")
    assert caught.value.code == code
    assert len(requests) == 1
    assert requests[0].full_url == "http://127.0.0.1:8010/api/state"
    assert requests[0].get_header("Authorization") == "Bearer placeholder-launch-token"


def test_launcher_still_sends_authenticated_direct_json_request(monkeypatch):
    headers = Message()
    headers["Content-Type"] = "application/json"
    requests = fake_transport(monkeypatch, lambda request: (200, headers, b'{"armed": false}'))
    response = launch.request_json("http://127.0.0.1:8010", "/api/disarm", "placeholder-launch-token", {})
    assert response == {"armed": False}
    assert len(requests) == 1
    assert requests[0].get_method() == "POST"
    assert json.loads(requests[0].data) == {}

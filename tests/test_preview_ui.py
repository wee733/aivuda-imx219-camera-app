"""Run the actual preview page in headless Chrome, with only the HTTP API stubbed."""
import json
from pathlib import Path
import shutil
import subprocess

import pytest


UI = Path(__file__).resolve().parents[1] / 'app/ui/index.html'


@pytest.mark.parametrize('codec, backend, initial, fps', [
    ('auto', 'gst_nvenc', 'video', 29.97),
    ('nvenc_h264', 'ffmpeg_nvenc', 'video', 7),
    ('jpeg', 'gst_nvenc', 'img', 15),
    ('auto', 'ffmpeg_libx264', 'img', 1),
    ('nvenc_h264', None, 'img', 30),
])
def test_initial_stream_switch_and_settings(tmp_path, codec, backend, initial, fps):
    chrome = shutil.which('google-chrome') or shutil.which('chromium')
    if not chrome:
        pytest.skip('headless Chrome/Chromium unavailable')
    status = {'settings': {'codec': codec, 'fps': fps, 'width': 319, 'jpeg_quality': 84},
              'capabilities': {'h264_stream_backend': backend,
                               'h264_hardware': backend not in (None, 'ffmpeg_libx264')},
              'bitrate_kbps': 0}
    setup = '''
      let releaseStatus, posted;
      const firstStatus = new Promise(resolve => { releaseStatus = resolve; });
      window.fetch = async (url, options) => {
        if (url.endsWith('/settings')) posted = JSON.parse(options.body);
        return {ok: true, json: () => url.endsWith('/status') ? firstStatus : {}};
      };
    '''
    checks = '''
      const check = (condition, message) => { if (!condition) throw Error(message); };
      const media = () => document.querySelector('#stage [src]');
      (async () => {
        check(!media(), 'stream started before codec settings arrived');
        releaseStatus(statusData);
        await poll();
        const encoder = media();
        check(encoder?.localName === initial, 'wrong initial codec');
        const hasBackend = Boolean(statusData.capabilities.h264_stream_backend);
        check(($('modeBtn').style.display !== 'none') === hasBackend, 'incorrect switch availability');
        if (hasBackend) {
          $('modeBtn').click();
          const switched = media();
          check(switched && switched.localName !== initial, 'manual switch failed');
          await poll();
          check(switched === media(), 'poll reset or restarted stream');
          $('toggleBtn').click();
          check(!media(), 'pause retained stream');
          $('toggleBtn').click();
          check(media()?.localName === switched.localName, 'resume lost codec choice');
          if (media().localName !== 'video') $('modeBtn').click();
          media().dispatchEvent(new Event('error'));
          check(media()?.localName === 'img', 'video error did not restore MJPEG');
        }
        const {width, fps} = statusData.settings;
        check($('widthSel').value === String(width) && $('fpsSel').value === String(fps),
              'select lost configured value');
        $('qRange').value = '90';
        $('applyBtn').click();
        check(posted?.width === width && posted?.fps === fps && posted?.jpeg_quality === 90,
              'applying quality changed width or fps');
        document.body.textContent = 'PREVIEW TEST PASSED';
      })().catch(error => { document.body.textContent = 'PREVIEW TEST FAILED: ' + error.message; });
    '''
    page = UI.read_text().replace('<head>', '<head><meta http-equiv="Content-Security-Policy" '
                                  'content="img-src \'none\'; media-src \'none\'; connect-src \'none\'">')
    page = page.replace('<script>', f'<script>{setup}</script><script>', 1)
    page = page.replace('</body>', f'<script>const statusData={json.dumps(status)}, '
                                   f'initial={json.dumps(initial)};{checks}</script></body>')
    path = tmp_path / 'index.html'
    path.write_text(page)
    run = subprocess.run([chrome, '--headless', '--disable-background-networking', '--disable-gpu',
                          f'--user-data-dir={tmp_path / "profile"}', '--dump-dom', path.as_uri()],
                         text=True, capture_output=True, check=True, timeout=20)
    result = run.stdout.partition('<body>')[2].partition('</body>')[0].strip()
    assert result == 'PREVIEW TEST PASSED'

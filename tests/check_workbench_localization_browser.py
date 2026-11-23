"""Independent HTTP/browser check; synthetic owner snapshots, no device requests."""
import asyncio,json,sys,tempfile
from pathlib import Path
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application
from workbench.daemon import localization_progress

async def main():
 with tempfile.TemporaryDirectory(prefix='static-startup-ui-',dir=ROOT/'.runtime') as folder:
  app=application(Path(folder)/'no-owner.sock',folder);server=TestServer(app);await server.start_server()
  try:
   async with async_playwright() as p:
    browser=await p.chromium.launch(headless=True);page=await browser.new_page(viewport={'width':1600,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    s={'view':'live','demo':False,'device_control_available':True,'map_id':next(iter(app['api'].assets.maps)),
       'session':{'id':'test','state':'STARTING','stage':'relocalization'},'topics':{},'topic_receipt_age_ms':{},
       'feedback_stopped':False,'trace':[],'events':[],'recording':{},'task':None}
    def phase(code,metrics):
     detail=localization_progress(code,metrics);s['session']['reason']='等待重定位确认：'+detail
     s['session']['progress']={'components':{'relocalization':detail}}
    phase('searching_waiting_query_window',{'query_window_frames':7,'query_window_required_frames':8})
    await page.route('**/api/v1/snapshot',lambda r:r.fulfill(json=s));sockets=[]
    async def ws(w):sockets.append(w);w.send(json.dumps(s))
    await page.route_web_socket('**/api/v1/events',ws)
    await page.goto(str(server.make_url('/')));await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button')
    await page.wait_for_selector('#login',state='hidden');assert '7/8' in await page.locator('#sessionReason').text_content()
    phase('searching_geometric_registration',{'query_window_frames':8,'query_window_required_frames':8})
    sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    text=await page.locator('#sessionReason').text_content();assert '8/8' in text and '正在检索和几何匹配' in text and '7/8' not in text
    phase('verifying_stationary_support_restarted',{'support_queries':1,'support_elapsed_s':0.,'support_anchor_strong':True})
    sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '丢弃较弱初始候选' in await page.locator('#sessionReason').text_content();assert await page.locator('#execute').is_disabled()
    phase('verifying_wait_strong_stationary_geometry',{'best_rmse_m':.13,'stationary_max_rmse_m':.12})
    sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '0.130m' in await page.locator('#sessionReason').text_content();assert not errors,errors
    out=ROOT/'docs/workbench_evidence/20261117/static_confirmation'
    await page.screenshot(path=str(out/'ui.png'),full_page=True)
    (out/'browser.json').write_text(json.dumps({'pass':True,'console_errors':errors,'no_device_requests':True,
       'checks':['7_to_8_matching_phase','fresh_strong_anchor_count','weak_geometry_reason','goals_disabled_during_search']},indent=2)+'\n')
    await browser.close();print('Static localization UI PASS; no device request')
  finally:await server.close()
asyncio.run(main())

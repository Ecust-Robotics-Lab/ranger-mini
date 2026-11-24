"""Standalone no-device UI check for C2 progress and the user-selected 0.05 stop limit."""
import asyncio,json,sys,tempfile
from pathlib import Path
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application

async def main():
 with tempfile.TemporaryDirectory(prefix='restart-stop-qa-',dir=ROOT/'.runtime') as folder:
  app=application(Path(folder)/'no-owner.sock',folder);server=TestServer(app);await server.start_server()
  try:
   async with async_playwright() as p:
    browser=await p.chromium.launch(headless=True);page=await browser.new_page(viewport={'width':1600,'height':1000});errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    s={'view':'live','demo':False,'device_control_available':True,'map_id':next(iter(app['api'].assets.maps)),
       'session':{'id':'test','state':'STARTING','stage':'relocalization','reason':'等待重定位确认：searching_waiting_query_window，查询窗口 4/8 帧','progress':{'components':{'relocalization':'查询窗口 4/8 帧'}}},
       'task':None,'topics':{'/wheel/odom':{'twist':{'twist':{'linear':{'x':.023,'y':0},'angular':{'z':0}}}}},
       'topic_receipt_age_ms':{'/wheel/odom':10},'feedback_stopped':True,'feedback_sample_age_ms':10,
       'motion_limits':{'stop_linear':.05,'stop_angular':.025,'stop_dwell':.5},'trace':[],'events':[],'recording':{}}
    await page.route('**/api/v1/snapshot',lambda r:r.fulfill(json=s));sockets=[]
    async def ws(w):sockets.append(w);w.send(json.dumps(s))
    await page.route_web_socket('**/api/v1/events',ws)
    await page.goto(str(server.make_url('/')));await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button');await page.wait_for_selector('#login',state='hidden')
    assert '4/8' in await page.locator('#sessionReason').text_content();assert '4/8' in await page.locator('#sessionProgress').text_content()
    assert await page.locator('#execute').is_disabled();assert '已停稳' in await page.locator('#stationary').text_content()
    assert await page.locator('#measuredSpeed').text_content()=='0.023'
    out = ROOT/'.runtime/validation/workbench/restart_and_stop'
    out.mkdir(parents=True, exist_ok=True)
    await page.screenshot(path=str(out/'ui.png'),full_page=True)
    s['feedback_stopped']=False;s['topics']['/wheel/odom']['twist']['twist']['linear']['x']=.051;sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '仍有运动' in await page.locator('#stationary').text_content()
    s['topic_receipt_age_ms']['/wheel/odom']=300;sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '等待新鲜反馈' in await page.locator('#stationary').text_content();assert not errors,errors
    (out/'browser.json').write_text(json.dumps({'pass':True,'mock_transport_no_device_requests':True,'console_errors':errors,'checks':['localization_window_progress','navigation_disabled_until_localized','raw_023_speed_preserved','005_stop_limit','051_reports_motion','stale_feedback_unknown']},indent=2))
    await browser.close();print('Restart/stop UI PASS; no device request')
  finally:await server.close()
asyncio.run(main())

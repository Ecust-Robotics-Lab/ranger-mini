"""No-device page QA for 400ms localization and concrete endpoint errors."""
import asyncio,json,sys,tempfile
from pathlib import Path
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application

async def main():
 with tempfile.TemporaryDirectory(prefix='goal-freshness-ui-',dir=ROOT/'.runtime') as folder:
  app=application(Path(folder)/'no-owner.sock',folder);server=TestServer(app);await server.start_server()
  try:
   async with async_playwright() as p:
    browser=await p.chromium.launch(headless=True);page=await browser.new_page(viewport={'width':1600,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    s={'view':'live','demo':False,'device_control_available':True,'map_id':next(iter(app['api'].assets.maps)),
       'session':{'id':'test','state':'READY','stage':'ready','reason':'设备已启动；低延迟三融合前端已加载'},
       'localization':{'valid':True,'contract_valid':True,'state':1,'sample_age_ms':350,'receipt_age_ms':20,'body_pose':[-.73,-1.01,1.54]},
       'freshness_limits':{'localization_sample_max_age':400,'localization_receipt_max_age':250},
       'topics':{'/system_state':{'control_mode':1,'vehicle_state':0,'error_code':0}},
       'topic_receipt_age_ms':{'/system_state':10},'trace':[],'events':[],'recording':{},
       'task':{'id':'failed','state':'FAILED','pose':[1.985,.624,.405],
          'reason':'goal_live_obstacle_collision','terminal_reason':'no_progress_timeout'}}
    await page.route('**/api/v1/snapshot',lambda r:r.fulfill(json=s));sockets=[]
    async def ws(w):sockets.append(w);w.send(json.dumps(s))
    await page.route_web_socket('**/api/v1/events',ws)
    await page.goto(str(server.make_url('/')));await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button')
    await page.wait_for_selector('#login',state='hidden')
    assert await page.locator('#localization').text_content()=='有效'
    assert '400' in await page.locator('#sampleAge').get_attribute('title')
    assert '目标车体安全范围内存在实时障碍' in await page.locator('#taskReason').text_content()
    assert '低延迟三融合' in await page.locator('#sessionReason').text_content()
    await page.fill('#goalX','1.985');await page.fill('#goalY','0.624');await page.fill('#goalYaw','23.2')
    await page.locator('#goalYaw').dispatch_event('change')
    out=ROOT/'.runtime/validation/workbench/goal_freshness'
    out.mkdir(parents=True, exist_ok=True)
    await page.screenshot(path=str(out/'ui.png'),full_page=True)
    s['localization'].update(valid=False,state=2);sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '几何匹配暂退化' in await page.locator('#localization').text_content();assert await page.locator('#execute').is_disabled()
    s['localization'].update(state=1,sample_age_ms=401);sockets[0].send(json.dumps(s));await page.wait_for_timeout(120)
    assert '定位采样过期' in await page.locator('#localization').text_content();assert not errors,errors
    (out/'browser.json').write_text(json.dumps({'pass':True,'console_errors':errors,'no_device_requests':True,
       'checks':['350ms_pose_valid','400ms_bound_visible','concrete_goal_obstacle_reason','frontend_loaded_label','degraded_and_expired_distinct','goal_body_draw']},indent=2)+'\n')
    await browser.close();print('Goal/freshness UI PASS; no device request')
  finally:await server.close()
asyncio.run(main())

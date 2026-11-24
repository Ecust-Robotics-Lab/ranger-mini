"""Real allowlisted HTTP reset + browser state, synthetic backend only."""
import asyncio,json,sys,tempfile
from pathlib import Path
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application

async def main():
 with tempfile.TemporaryDirectory(prefix='reset-ui-',dir=ROOT/'.runtime') as folder:
  app=application(Path(folder)/'unused.sock',folder);calls=[]
  snapshot={'view':'live','demo':False,'device_control_available':True,'map_id':next(iter(app['api'].assets.maps)),
   'session':{'id':'owned','state':'READY','reason':'设备已启动；低延迟三融合前端已加载'},
   'task':{'id':'failed','state':'FAILED','reason':'start_live_obstacle_collision'},
   'localization':{'valid':False,'state':2,'body_pose':[-.73,-1.,1.54]},
   'navigation_reset':{'state':'IDLE'},'topics':{'/navigation/state':{'state':'FAILED','worker':{'ready':True}},
     '/system_state':{'control_mode':1,'vehicle_state':0,'error_code':0}},
   'topic_receipt_age_ms':{'/system_state':10},'trace':[],'events':[],'recording':{}}
  async def rpc(method,data=None):
   if method=='snapshot':return snapshot
   assert method=='navigation.reset' and data['view']=='live' and data['session_id']=='owned'
   calls.append(method);snapshot['navigation_reset']={'state':'RESETTING','reason':'正在初始化规划控制；定位继续运行'}
   snapshot['topics']['/navigation/state']={'state':'RESETTING','worker':{'ready':False}}
   return snapshot['navigation_reset']
  app['api'].rpc=rpc;server=TestServer(app);await server.start_server()
  try:
   async with async_playwright() as p:
    browser=await p.chromium.launch(headless=True);page=await browser.new_page(viewport={'width':1600,'height':1100});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    await page.goto(str(server.make_url('/')));await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button')
    await page.wait_for_selector('#login',state='hidden')
    assert await page.locator('#cancel').is_disabled();assert await page.locator('#execute').is_disabled()
    assert await page.locator('#resetNavigation').is_enabled()
    out=ROOT/'.runtime/validation/workbench/navigation_reset'
    out.mkdir(parents=True, exist_ok=True)
    await page.screenshot(path=str(out/'failed_reset_enabled.png'),full_page=True)
    await page.click('#resetNavigation');await page.wait_for_timeout(220)
    assert calls==['navigation.reset'];assert await page.locator('#resetNavigation').is_disabled()
    assert '重置中' in await page.locator('#taskState').text_content();assert await page.locator('#execute').is_disabled()
    snapshot['task']=None;snapshot['navigation_reset']={'state':'READY','reason':'规划控制已就绪'}
    snapshot['topics']['/navigation/state']={'state':'IDLE','reason':'planning_control_reset','worker':{'ready':True}}
    snapshot['localization'].update(valid=True,state=1);await page.wait_for_timeout(220)
    assert '等待新目标' in await page.locator('#taskState').text_content();assert await page.locator('#execute').is_enabled()
    assert await page.locator('#resetNavigation').is_enabled();assert calls==['navigation.reset']
    snapshot['navigation_reset']={'state':'FAILED','reason':'初始化超时，可重试'}
    snapshot['topics']['/navigation/state']={'state':'RESETTING','worker':{'ready':False}}
    await page.wait_for_timeout(220);assert await page.locator('#resetNavigation').is_enabled()
    assert await page.locator('#execute').is_disabled();assert '重置失败' in await page.locator('#taskState').text_content()
    snapshot['demo']=True;await page.wait_for_timeout(220);assert await page.locator('#resetNavigation').is_disabled()
    assert not errors,errors
    (out/'browser.json').write_text(json.dumps({'pass':True,'no_device_requests':True,'console_errors':errors,
       'checks':['failed_and_invalid_pose_can_reset','reset_uses_allowlisted_http','busy_shows_progress_and_blocks_goal','ready_clears_old_task','never_auto_sends_goal','failed_reset_can_retry','demo_readonly']},indent=2)+'\n')
    await browser.close();print('Reset UI/HTTP PASS; synthetic backend only')
  finally:await server.close()
asyncio.run(main())

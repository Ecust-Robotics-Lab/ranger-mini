"""Isolated browser QA of CAN/RC/standby feedback; no ROS owner or device requests."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application

async def main():
 with tempfile.TemporaryDirectory(prefix='workbench-control-qa-',dir=ROOT/'.runtime') as folder:
  app=application(Path(folder)/'nonexistent-owner.sock',folder);server=TestServer(app);await server.start_server()
  try:
   async with async_playwright() as p:
    browser=await p.chromium.launch(headless=True);page=await browser.new_page(viewport={'width':1600,'height':1000});errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    map_id=next(iter(app['api'].assets.maps))
    snapshot={'view':'live','demo':False,'device_control_available':True,'map_id':map_id,'session':{'id':'qa','state':'READY','reason':'底盘已反馈 CAN 控制模式'},'task':None,'topics':{'/system_state':{'control_mode':1,'vehicle_state':0,'error_code':0}},'topic_receipt_age_ms':{'/system_state':10},'localization':{'valid':True,'body_pose':[0,0,0]},'trace':[],'events':[],'recording':{}}
    await page.route('**/api/v1/snapshot',lambda r:r.fulfill(json=snapshot));sockets=[]
    async def socket(ws):sockets.append(ws);ws.send(json.dumps(snapshot))
    await page.route_web_socket('**/api/v1/events',socket)
    await page.goto(str(server.make_url('/')));await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button');await page.wait_for_selector('#login',state='hidden')
    assert 'CAN / 电脑控制' in await page.locator('#chassisControl').text_content();assert not await page.locator('#execute').is_disabled()
    snapshot['topics']['/system_state']['control_mode']=3;sockets[0].send(json.dumps(snapshot));await page.wait_for_timeout(100)
    assert '遥控器优先' in await page.locator('#chassisControl').text_content();assert await page.locator('#execute').is_disabled()
    snapshot['topics']['/system_state']['control_mode']=0;sockets[0].send(json.dumps(snapshot));await page.wait_for_timeout(100)
    assert '待机' in await page.locator('#chassisControl').text_content();assert await page.locator('#execute').is_disabled();assert not errors,errors
    evidence=ROOT/'docs/workbench_evidence/20261117/computer_control';evidence.mkdir(exist_ok=True,parents=True)
    await page.screenshot(path=str(evidence/'ui.png'),full_page=True)
    (evidence/'browser.json').write_text(json.dumps({'pass':True,'no_remote_required':True,'CAN_feedback_enables_goal_button':True,'RC_and_standby_disable_goal_button':True,'console_errors':errors,'mock_transport_no_hardware_requests':True},indent=2))
    await browser.close();print('Computer-control display PASS; mock transport, no hardware request')
  finally:await server.close()
asyncio.run(main())

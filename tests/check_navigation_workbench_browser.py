"""Local browser QA. Requires the --demo server; never operates hardware."""
import asyncio
import json
from pathlib import Path
import sys
from playwright.async_api import async_playwright
ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'docs/workbench_evidence/20261117'

async def main():
 async with async_playwright() as p:
  browser=await p.chromium.launch(headless=True)
  page=await browser.new_page(viewport={'width':1600,'height':1000});errors=[]
  page.on('pageerror',lambda e:errors.append(str(e)))
  await page.goto('http://127.0.0.1:8080')
  await page.fill('#accessKey',(ROOT/'.runtime/navigation_workbench/access.key').read_text().strip())
  await page.click('#loginForm button');await page.wait_for_selector('#login',state='hidden')
  await page.wait_for_function("()=>document.querySelector('#connection').textContent==='工控机已连接'")
  assert await page.locator('#demoBadge').is_visible()
  assert await page.locator('#startSession').is_disabled()
  box=await page.locator('#map').bounding_box()
  original=await page.locator('#goalX').input_value()
  await page.mouse.move(box['x']+box['width']*.6,box['y']+box['height']*.5);await page.mouse.down();await page.mouse.move(box['x']+box['width']*.65,box['y']+box['height']*.5);await page.mouse.up()
  assert original!=await page.locator('#goalX').input_value()
  # Independently exercise rotated map conversions in the actual browser module.
  valid=await page.evaluate("""async()=>{const {localToWorld,worldToLocal}=await import('/assets/map.js');const p=localToWorld(2,3,[7,11,Math.PI/2]);const q=worldToLocal(...p,[7,11,Math.PI/2]);return Math.abs(p[0]-4)<1e-9&&Math.abs(p[1]-13)<1e-9&&Math.abs(q[0]-2)<1e-9&&Math.abs(q[1]-3)<1e-9;}""")
  assert valid
  await page.locator('#offlineStart summary').click()
  for k,v in {'#startX':'10','#startY':'2.25','#startYaw':'0','#goalX':'10.5','#goalY':'2.25','#goalYaw':'0'}.items():
   await page.fill(k,v);await page.locator(k).dispatch_event('change')
  await page.click('#preview');await page.wait_for_function("()=>document.querySelector('#preview').disabled===false",timeout=20000)
  preview=await page.locator('#previewResult').text_content();assert '静态扫掠通过' in preview,preview
  await page.screenshot(path=str(EVIDENCE/'desktop.png'),full_page=True)
  await page.set_viewport_size({'width':820,'height':1180});await page.screenshot(path=str(EVIDENCE/'tablet.png'),full_page=True)
  await page.set_viewport_size({'width':390,'height':844});await page.screenshot(path=str(EVIDENCE/'mobile.png'),full_page=True)
  assert await page.evaluate('document.documentElement.scrollWidth<=innerWidth')
  await page.set_viewport_size({'width':1600,'height':1000})
  await page.click('#replayTab');await page.select_option('#recordSelect','recorded-u622df4n-v4');await page.click('#loadRecord')
  await page.wait_for_function("()=>document.querySelector('#seek').max>100")
  await page.locator('#seek').fill('100');await page.locator('#seek').dispatch_event('input');await page.wait_for_timeout(350)
  assert await page.locator('#execute').is_disabled();assert '1:40' in await page.locator('#replayTime').text_content()
  await page.screenshot(path=str(EVIDENCE/'replay.png'),full_page=True)
  await page.click('#play');await page.wait_for_timeout(500);await page.click('#play')
  await page.reload();await page.wait_for_function("()=>document.querySelector('#connection').textContent==='工控机已连接'")
  assert await page.locator('#login').is_hidden()
  assert not errors,errors
  result={'pass':True,'console_errors':errors,'checks':['login','actual_map','demo_hardware_disabled','map_target_drag','rotated_map_coordinate_roundtrip','static_planning','desktop_tablet_mobile_layout','recorded_bag_seek','replay_readonly','play_pause','refresh_reconnect'],'preview':preview}
  (EVIDENCE/'browser.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False))
  await browser.close()
asyncio.run(main())

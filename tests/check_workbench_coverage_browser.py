"""Coverage envelope/unknown-cell explanations through real HTTP and browser."""
import argparse,asyncio,json,sys,tempfile
from pathlib import Path
from aiohttp.test_utils import TestServer
from playwright.async_api import async_playwright,expect
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.server import application

async def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-directory',type=Path,default=ROOT/'docs/workbench_evidence/20261117/coverage_margin')
    out=parser.parse_args().evidence_directory;out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='coverage-browser-',dir=ROOT/'.runtime') as folder:
        app=application(Path(folder)/'unused.sock',folder)
        diagnostic={'accepted':False,'total_margin_m':.025,'protected_margin_m':.015,
                    'unknown_cell_budget':8,'unknown_edge_cells':2,'unknown_protected_cells':1,
                    'reason':'coverage_body_unobserved'}
        nav={'state':'OBSERVING','reason':'guidance_unobserved','worker':{'ready':True},
             'coverage_diagnostic':{'current':diagnostic,'blocked':None},'control_limits':{'max_speed':.30,'max_yaw_rate':.50}}
        snapshot={'view':'live','demo':False,'device_control_available':True,'map_id':next(iter(app['api'].assets.maps)),
                  'session':{'id':'synthetic','state':'READY'},'task':{'state':'RUNNING'},
                  'localization':{'valid':True,'state':1,'body_pose':[.327,-.055,.021]},
                  'topics':{'/navigation/state':nav},'trace':[],'events':[],'recording':{}}
        async def rpc(method,data=None):
            assert method=='snapshot','read-only test must not request a control action'
            return snapshot
        app['api'].rpc=rpc;server=TestServer(app);await server.start_server()
        try:
            async with async_playwright() as p:
                browser=await p.chromium.launch(headless=True)
                page=await browser.new_page(viewport={'width':1600,'height':1100});errors=[]
                page.on('pageerror',lambda e:errors.append(str(e)))
                await page.goto(str(server.make_url('/')))
                await page.fill('#accessKey',app['api'].key);await page.click('#loginForm button')
                await page.wait_for_selector('#login',state='hidden')
                await expect(page.locator('#taskReason')).to_contain_text('总外扩 25mm')
                text=await page.locator('#taskReason').text_content()
                assert '保护范围未知 1/0 格' in text and '边缘未知 2/8 格' in text
                await expect(page.locator('#controlSpeedLimit')).to_contain_text('0.30 m/s')
                await page.screenshot(path=str(out/'coverage_diagnostic_browser.png'),full_page=True)
                nav.update(state='FAILED',reason='no_progress_timeout',failure_cause='guidance_unobserved')
                snapshot['task']={'state':'FAILED','reason':'guidance_unobserved'}
                diagnostic.update(reason='coverage_edge_budget_exceeded',unknown_protected_cells=2,
                                  protected_unknown_cell_budget=2,unknown_edge_cells=9)
                await expect(page.locator('#taskReason')).to_contain_text('边缘未知格超过容许数量')
                await expect(page.locator('#taskReason')).to_contain_text('保护范围未知 2/2 格')
                nav.update(state='IDLE',reason='planning_control_reset',failure_cause=None)
                snapshot['task']=None
                await expect(page.locator('#taskReason')).not_to_contain_text('总外扩')
                assert not errors,errors
                (out/'browser.json').write_text(json.dumps({'passed':True,'no_control_requests':True,
                    'checks':['total_25mm','protected_and_edge_counts','actual_navigation_speed_cap','failed_reason_keeps_detail','idle_hides_old_coverage'],
                    'console_errors':errors},indent=2)+'\n')
                await browser.close()
                print('Coverage browser PASS; no control requests')
        finally:await server.close()

asyncio.run(main())

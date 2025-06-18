from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import pandas as pd
import time
import random

search_url = "https://scholar.google.com/scholar?q=Software+Design+and+Verification+for+Robotics+in+the+Nuclear+Sector&hl=en&as_sdt=0%2C5&as_ylo=2020&as_yhi=2024"
num_pages = 60
list_condition_1 = ["software", "program", "develop"]
list_condition_2 = ["verification", "testing", "simulation"]
list_condition_3 = ["nuclear", "decommissioning"]
list_condition_4 = ["robot", "system"]


"""Set up browser"""
def setup_browser():
    chrome_options = Options()
    
    # Add browser options
    chrome_options.add_argument("--no-sandbox")
    chrome_options.add_argument("--disable-dev-shm-usage")
    chrome_options.add_argument("--disable-blink-features=AutomationControlled")
    chrome_options.add_experimental_option("excludeSwitches", ["enable-automation"])
    chrome_options.add_experimental_option('useAutomationExtension', False)
    
    # Mimic real user browser
    chrome_options.add_argument("--user-agent=Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36")
    chrome_options.add_argument("--window-size=1366,768")
    chrome_options.add_argument("--start-maximized")
    
    driver = webdriver.Chrome(options=chrome_options)
    
    # Execute browser scripts
    driver.execute_script("Object.defineProperty(navigator, 'webdriver', {get: () => undefined})")
    
    return driver

"""Go to Google Scholar homepage first to get cookies and session"""
def initialise_google_scholar(driver):
    print("Visiting Google Scholar homepage to establish session...")
    driver.get("https://scholar.google.com")
    
    # Wait and simulate human behavior
    time.sleep(random.uniform(3, 6))
    
    # Scroll a bit to simulate human browsing
    driver.execute_script("window.scrollBy(0, 200);")
    time.sleep(random.uniform(1, 3))
    driver.execute_script("window.scrollBy(0, -100);")
    time.sleep(random.uniform(1, 2))
    
    print("✅ Session established with Google Scholar")

"""Navigate to your specific search URL"""
def navigate_to_search_url(driver, search_url):
    print("Navigating to your search URL...")
    print(f"URL: {search_url[:80]}...")
    
    driver.get(search_url)
    
    # Wait for results to load
    time.sleep(random.uniform(5, 8))
    
    # Check if we got redirected or blocked
    current_url = driver.current_url
    if "scholar.google.com/scholar" not in current_url:
        print(f"⚠️ Possible redirect detected. Current URL: {current_url[:50]}...")
        return False
    
    print("✅ Successfully loaded search results")
    return True

"""Scrape results from the current page"""
def scrape_current_page(driver, page_num):
    results = []
    
    try:
        print(f"Extracting results from page {page_num}...")
        
        # Wait for results to be present
        try:
            WebDriverWait(driver, 10).until(
                EC.presence_of_element_located((By.CLASS_NAME, "gs_ri"))
            )
        except:
            print("❌ No results found on this page")
            return []
        
        papers = driver.find_elements(By.CLASS_NAME, "gs_ri")
        print(f"Found {len(papers)} papers on page {page_num}")
        
        for i, paper in enumerate(papers, 1):
            try:
                # Scroll to paper
                driver.execute_script("arguments[0].scrollIntoView(true);", paper)
                time.sleep(random.uniform(0.8, 1.5))
                
                # Highlight current paper being processed
                driver.execute_script("arguments[0].style.border='2px solid blue'", paper)
                
                # Get title
                try:
                    title_elem = paper.find_element(By.CSS_SELECTOR, "h3.gs_rt")
                    title = title_elem.text.strip()
                    print(f"  {i}. {title[:60]}...")
                except:
                    title = "No title"
                    print(f"  {i}. No title found")
                
                # Get snippet
                try:
                    snippet_elem = paper.find_element(By.CSS_SELECTOR, "div.gs_rs")
                    snippet = snippet_elem.text.strip()
                    print(f"      {snippet[:80]}...")
                except:
                    snippet = "No snippet"
                    print(f"      No snippet")
                
                # Get authors
                try:
                    author_elem = paper.find_element(By.CSS_SELECTOR, "div.gs_a")
                    authors = author_elem.text.strip()
                except:
                    authors = "No authors"
                
                # Get link
                try:
                    link_elem = title_elem.find_element(By.TAG_NAME, "a")
                    link = link_elem.get_attribute("href")
                except:
                    link = "No link"
                
                # Check if paper matches the keywords
                text_to_check = f"{title} {snippet}".lower()
                
                # Boolean logic based on new query:
                condition1 = any(keyword in text_to_check for keyword in list_condition_1)
                
                condition2 = any(keyword in text_to_check for keyword in list_condition_2)
                
                condition3 = any(keyword in text_to_check for keyword in list_condition_3)
                
                condition4 = any(keyword in text_to_check for keyword in list_condition_4)
                
                matches = (condition1 or condition2) and condition3 and condition4
                
                # Visual feedback
                if matches:
                    driver.execute_script("arguments[0].style.border='3px solid green'", paper)
                    reasons = []
                    if condition1: reasons.append("OR".join(list_condition_1))
                    if condition2: reasons.append("OR".join(list_condition_2))
                    if condition3: reasons.append("OR".join(list_condition_3)) 
                    if condition4: reasons.append("OR".join(list_condition_4)) 
                    print(f"     ✅ MATCH! ({'; '.join(reasons)})")
                else:
                    driver.execute_script("arguments[0].style.border='3px solid orange'", paper)
                    print(f"     ❌ No match")
                
                # Store result
                results.append({
                    'Page': page_num,
                    'Position': i,
                    'Title': title,
                    'Snippet': snippet,
                    'Authors': authors,
                    'Link': link,
                    'Matches_Keywords': matches,
                    'Condition1': condition1,
                    'Condition2': condition2,
                    'Condition3': condition3,
                    'Condition4': condition4
                })
                
                # Brief pause to show result
                time.sleep(random.uniform(1, 2))
                
                # Remove border
                driver.execute_script("arguments[0].style.border=''", paper)
                
            except Exception as e:
                print(f"  ❌ Error processing paper {i}: {e}")
                continue
        
        return results
        
    except Exception as e:
        print(f"❌ Error scraping page {page_num}: {e}")
        return []

"""Navigate to the next page by modifying the URL"""
def go_to_next_page(driver, current_page):
    try:
        print(f"Going to page {current_page + 1}...")
        
        current_url = driver.current_url
        
        # Calculate start parameter for next page (current_page is 1-indexed, so subtract 1)
        next_start = current_page * 10
        
        # Modify URL for next page
        if "start=" in current_url:
            # Replace existing start parameter
            import re
            new_url = re.sub(r'start=\d+', f'start={next_start}', current_url)
        else:
            # Add start parameter
            separator = "&" if "?" in current_url else "?"
            new_url = f"{current_url}{separator}start={next_start}"
        
        print(f"Loading: {new_url[:80]}...")
        driver.get(new_url)
        
        # Wait for new page to load
        time.sleep(random.uniform(5, 8))
        
        return True
        
    except Exception as e:
        print(f"❌ Error navigating to next page: {e}")
        return False

"""Check if Google has blocked or redirected us"""
def check_for_blocking(driver):
    page_source = driver.page_source.lower()
    current_url = driver.current_url
    
    if ("unusual traffic" in page_source or 
        "captcha" in page_source or 
        "blocked" in page_source or
        "scholar.google.com/scholar" not in current_url):
        
        print("DETECTION: Google may have blocked or redirected us")
        print(f"Current URL: {current_url}")
        
        if "captcha" in page_source:
            print("CAPTCHA detected!")
            choice = input("Solve CAPTCHA manually and press Enter to continue, or 'q' to quit: ")
            if choice.lower() == 'q':
                return False
        else:
            print("Waiting 60 seconds before continuing...")
            time.sleep(60)
        
        return True
    
    return True

"""Main scraping function"""
def main():    
    print("Google Scholar Scraper - Direct URL Navigation")
    print("=" * 60)
    print("Step 1: Visit homepage to get cookies")
    print("Step 2: Navigate to your search URL")
    print("Step 3: Solve CAPTCHA")
    print("Step 4: Scrape multiple pages")
    print()
    
    driver = setup_browser()
    all_results = []
    
    try:
        # Step 1: Initialise session with Google Scholar
        initialise_google_scholar(driver)
        
        # Step 2: Navigate to your search URL
        if not navigate_to_search_url(driver, search_url):
            print("❌ Failed to load search results")
            return
        
        # Check if we need to solve CAPTCHA or wait
        if not check_for_blocking(driver):
            return
        
        # Step 3: Scrape multiple pages
        max_pages = num_pages  # Adjust as needed
        
        for page in range(max_pages):
            print(f"\n === PAGE {page + 1} ===")
            
            # Check for blocking before each page
            if not check_for_blocking(driver):
                break
            
            # Scrape current page
            page_results = scrape_current_page(driver, page + 1)
            
            if not page_results:
                print("❌ No results found, stopping")
                break
            
            all_results.extend(page_results)
            
            # Go to next page (except for last page)
            # FIXED: Pass the current page number (1-indexed), not page + 1
            if page < max_pages - 1:
                if not go_to_next_page(driver, page + 1):
                    print("❌ Cannot go to next page, stopping")
                    break
                
                # Wait between pages
                wait_time = random.uniform(2, 8)
                print(f"⏳ Waiting {wait_time:.1f} seconds before next page...")
                time.sleep(wait_time)
        
        # Save results
        if all_results:
            # Save all results
            df = pd.DataFrame(all_results)
            df.to_csv("scholar_direct_url_results.csv", index=False)
            print(f"\nSaved {len(all_results)} total results to scholar_direct_url_results.csv")
            
            # Save only matching results
            matching = df[df['Matches_Keywords'] == True]
            if not matching.empty:
                matching.to_csv("scholar_direct_url_matching.csv", index=False)
                print(f"Saved {len(matching)} matching papers to scholar_direct_url_matching.csv")
                print(f"Match rate: {(len(matching)/len(all_results)*100):.1f}%")
                
                # Show breakdown by condition
                cond1_count = matching['Condition1'].sum()
                cond2_count = matching['Condition2'].sum()
                cond3_count = matching['Condition3'].sum()
                cond4_count = matching['Condition4'].sum()

                print(f"   Matches breakdown:")
                print(f"   Condition 1 (software OR design OR program): {cond1_count}")
                print(f"   Condition 2 (verification OR validation OR testing OR simulation): {cond2_count}")
                print(f"   Condition 3 (nuclear OR waste OR decommission): {cond3_count}")
                print(f"   Condition 4 (robot OR system): {cond4_count}")
            else:
                print("❌ No papers matched your criteria")
        else:
            print("❌ No results to save")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        input("\n Press Enter to close browser...")
        driver.quit()

if __name__ == "__main__":
    print("Google Scholar Direct URL Scraper")
    print("Install: pip install selenium pandas")
    print("This will:")
    print("   1. Visit Google Scholar homepage")
    print("   2. Navigate to your exact search URL") 
    print("   3. Scrape results with visual feedback")
    print()
    
    main()
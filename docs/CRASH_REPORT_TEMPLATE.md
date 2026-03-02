# Crash Report Template

> Every crash is a lesson. Document it or you'll repeat it.

**Date:** ___________  
**Flight #:** ___________  
**Project:** ___________

---

## Incident Summary
*One paragraph: what happened?*

## Timeline
| Time | Event |
|------|-------|
| 0:00 | Armed, hover at 1m |
| 0:15 | ... |
| 0:xx | Incident |

## Root Cause Analysis
### Immediate Cause
*What directly caused the crash? (e.g., motor failure, loss of control, pilot error)*

### Contributing Factors
*What conditions made the crash more likely? (e.g., wind, untested gains, low battery)*

### Root Cause
*What is the underlying issue? (e.g., PID gains not tested at high throttle, no wind compensation)*

## Damage Assessment
- Frame: 
- Props: 
- Motors: 
- Electronics: 
- Battery: 
- Total cost: $

## Flight Data
*Attach or link to:*
- [ ] Log file (.csv / .bin)
- [ ] Key plots (attitude, motor outputs, RC inputs around crash time)
- [ ] Video (if available)

## Corrective Actions
| Action | Status |
|--------|--------|
| *e.g., Reduce max angle to ±30°* | ☐ |
| *e.g., Add motor output saturation warning* | ☐ |
| *e.g., Test PID at 50%, 75%, 100% throttle in sim* | ☐ |

## Lessons Learned
*What will you do differently next time?*

---

## Preventive Measures Added
- [ ] Added to pre-flight checklist: ___________
- [ ] Code change committed: ___________
- [ ] Hardware modification: ___________
